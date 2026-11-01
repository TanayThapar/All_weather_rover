"""
Degradation-Aware Uncertainty & Covariance Estimation Neural Network (DA-Net).
Predicts:
1. Dynamic sensor reliability weights and noise covariance scale factors.
2. Heteroscedastic body velocity priors and aleatoric uncertainties for EKF fusion.

Includes both PyTorch neural network and lightweight NumPy inference fallback.
"""

from typing import Tuple, Dict, Any, Optional
import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    nn = object  # fallback dummy


if HAS_TORCH:
    class DegradationCovarianceNet(nn.Module):
        """
        Deep multi-modal temporal network for degradation-aware state estimation.
        Input: (Batch, Sequence_Length, In_Features)
        """
        def __init__(self,
                     in_features: int = 16,
                     hidden_dim: int = 64,
                     num_layers: int = 2):
            super().__init__()
            self.in_features = in_features
            self.hidden_dim = hidden_dim

            # Feature projection
            self.feature_encoder = nn.Sequential(
                nn.Linear(in_features, hidden_dim),
                nn.LayerNorm(hidden_dim),
                nn.GELU(),
                nn.Dropout(0.1)
            )

            # Temporal GRU
            self.temporal_gru = nn.GRU(
                input_size=hidden_dim,
                hidden_size=hidden_dim,
                num_layers=num_layers,
                batch_first=True,
                bidirectional=False
            )

            # Head 1: Sensor Confidence Weights [radar, lidar, camera, wheel] in (0, 1)
            self.head_weights = nn.Sequential(
                nn.Linear(hidden_dim, 32),
                nn.ReLU(),
                nn.Linear(32, 4),
                nn.Sigmoid()
            )

            # Head 2: Dynamic Covariance Scale Factors (alpha_radar, alpha_lidar, alpha_wheel)
            # Softplus ensures positive scale factor [0.1, 100]
            self.head_cov_scale = nn.Sequential(
                nn.Linear(hidden_dim, 32),
                nn.ReLU(),
                nn.Linear(32, 3),
                nn.Softplus()
            )

            # Head 3: Velocity Prior [vx, vy, vz] and log-variance [log_s_x, log_s_y, log_s_z]
            self.head_velocity = nn.Sequential(
                nn.Linear(hidden_dim, 32),
                nn.ReLU(),
                nn.Linear(32, 6)
            )

        def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
            """
            Args:
                x: (Batch, Seq_Len, In_Features) or (Batch, In_Features)
            Returns:
                weights: (Batch, 4) in [0, 1]
                cov_scale: (Batch, 3) > 0.1
                v_pred: (Batch, 3) in m/s
                v_logvar: (Batch, 3) log(sigma^2)
            """
            if x.dim() == 2:
                x = x.unsqueeze(1)

            B, T, D = x.shape
            enc = self.feature_encoder(x.view(B * T, D))
            enc = enc.view(B, T, -1)

            out_gru, _ = self.temporal_gru(enc)
            feat = out_gru[:, -1, :]  # Take final temporal step

            weights = self.head_weights(feat)
            cov_scale = self.head_cov_scale(feat) + 0.1
            vel_out = self.head_velocity(feat)

            v_pred = vel_out[:, 0:3]
            v_logvar = torch.clamp(vel_out[:, 3:6], min=-8.0, max=5.0)

            return weights, cov_scale, v_pred, v_logvar
else:
    class DegradationCovarianceNet:
        def __init__(self, *args, **kwargs):
            raise ImportError("PyTorch is required to initialize DegradationCovarianceNet. Run pip install torch.")


class NumPyDegradationPredictor:
    """
    Lightweight, dependency-free inference engine running on pure NumPy.
    Can load exported network weights from JSON or NPZ for embedded deployment on rover.
    """
    def __init__(self, weights_dict: Optional[Dict[str, np.ndarray]] = None):
        self.weights = weights_dict

    def predict(self, feature_vector: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Fallback heuristic predictor when neural network weights are not yet trained.
        Derives degradation metrics directly from optical and RF sensor statistics.

        feature_vector layout:
            [0:5] Radar: [tgt_count, mean_dop, std_dop, mean_snr, dispersion]
            [5:10] LiDAR: [pt_count, mean_range, range_var, backscatter_ratio, ring_entropy]
            [10:13] IMU: [acc_mag, acc_var, gyr_mag]
            [13:16] Wheel: [fwd_vel, left_rpm, right_rpm]
        """
        feat = np.asarray(feature_vector, dtype=np.float64).flatten()
        if len(feat) < 14:
            feat = np.pad(feat, (0, 16 - len(feat)))

        radar_count = feat[0]
        radar_snr = feat[3]
        lidar_backscatter = feat[8]  # Near-field backscatter (< 1.0m reflection from fog/dust)
        lidar_pt_count = feat[5]

        # 1. Degradation heuristic
        # If LiDAR backscatter is high and point count drops, severe fog/dust is detected
        fog_dust_severity = np.clip((lidar_backscatter * 2.0) + (1.0 - np.clip(lidar_pt_count / 800.0, 0, 1)), 0.0, 1.0)

        # LiDAR confidence drops as fog rises
        w_lidar = float(np.clip(1.0 - fog_dust_severity, 0.05, 1.0))
        # 4D Radar stays robust in fog
        w_radar = float(np.clip(0.4 + (radar_snr / 20.0), 0.2, 1.0))
        w_cam = float(np.clip(1.0 - fog_dust_severity * 0.9, 0.1, 1.0))
        w_wheel = 0.95

        weights = np.array([w_radar, w_lidar, w_cam, w_wheel], dtype=np.float64)

        # 2. Covariance scaling factors
        cov_scale = np.array([
            float(np.clip(1.0 / w_radar, 0.5, 5.0)),
            float(np.clip(1.0 / (w_lidar ** 1.5), 1.0, 30.0)),
            float(np.clip(1.0 / w_wheel, 0.8, 3.0))
        ], dtype=np.float64)

        # 3. Direct velocity estimate from radar + wheel
        fwd_wheel = feat[13]
        mean_radar_dop = feat[1]
        v_fwd = 0.6 * fwd_wheel + 0.4 * abs(mean_radar_dop)
        v_pred = np.array([v_fwd, 0.0, 0.0], dtype=np.float64)
        v_logvar = np.array([-4.0, -4.0, -4.0], dtype=np.float64)

        return weights, cov_scale, v_pred, v_logvar
