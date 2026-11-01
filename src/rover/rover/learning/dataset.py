"""
Dataset Loader for Personal Live Recordings and Multi-Sensor Telemetry.
Handles:
1. Multi-session NPZ and CSV recordings.
2. Temporal sequence sliding windows with boundary preservation.
3. Feature normalization with exportable scaling statistics.
4. Data augmentation with synthetic atmospheric degradation injection.
"""

import os
import glob
import json
from typing import List, Tuple, Dict, Optional, Union
import numpy as np

try:
    import torch
    from torch.utils.data import Dataset
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    Dataset = object


class RoverPersonalDataset(Dataset):
    """
    PyTorch Dataset for personal live recordings from All Weather Rover.
    """

    FEATURE_NAMES = [
        # Radar (0:5)
        'radar_target_count', 'radar_mean_doppler', 'radar_std_doppler', 'radar_mean_snr', 'radar_dispersion',
        # LiDAR (5:10)
        'lidar_point_count', 'lidar_mean_range', 'lidar_range_std', 'lidar_backscatter_ratio', 'lidar_entropy',
        # IMU (10:13)
        'imu_accel_mag', 'imu_accel_var', 'imu_gyro_mag',
        # Wheel (13:16)
        'wheel_speed_vx', 'wheel_omega_z', 'wheel_steering_angle'
    ]

    def __init__(self,
                 data_source: Union[str, List[str]],
                 seq_len: int = 20,
                 stride: int = 2,
                 normalize: bool = True,
                 norm_params: Optional[Dict[str, np.ndarray]] = None,
                 augment_degradation: bool = True):
        """
        Args:
            data_source: Path to .npz/.csv file, list of files, or directory containing session files.
            seq_len: Number of temporal steps in sliding window (e.g. 20 steps = 1 sec @ 20Hz).
            stride: Stride between successive sliding windows.
            normalize: Whether to apply z-score normalization.
            norm_params: Optional pre-computed mean/std dict for test/eval sets.
            augment_degradation: Synthetically simulate fog/dust degradation on random batches.
        """
        super().__init__()
        self.seq_len = seq_len
        self.stride = stride
        self.augment_degradation = augment_degradation
        self.normalize = normalize

        # Discover all session recording files
        self.file_paths = self._resolve_file_paths(data_source)
        if not self.file_paths:
            raise FileNotFoundError(f"No valid .npz or .csv recording files found at: {data_source}")

        # Load and parse all sessions
        self.sessions = []
        for path in self.file_paths:
            session = self._load_session(path)
            if session is not None and len(session['features']) >= self.seq_len:
                self.sessions.append(session)

        if not self.sessions:
            raise ValueError(f"All recording files were empty or shorter than seq_len={seq_len}.")

        # Compute or apply normalization
        if norm_params is not None:
            self.mean = np.asarray(norm_params['mean'], dtype=np.float32)
            self.std = np.asarray(norm_params['std'], dtype=np.float32)
        elif self.normalize:
            all_feats = np.concatenate([s['features'] for s in self.sessions], axis=0)
            self.mean = np.mean(all_feats, axis=0).astype(np.float32)
            self.std = np.std(all_feats, axis=0).astype(np.float32)
            self.std[self.std < 1e-4] = 1.0  # Prevent zero division
        else:
            self.mean = np.zeros(16, dtype=np.float32)
            self.std = np.ones(16, dtype=np.float32)

        # Index valid window slices across all sessions
        self.window_indices: List[Tuple[int, int]] = []
        for s_idx, session in enumerate(self.sessions):
            n_steps = len(session['features'])
            for start_idx in range(0, n_steps - self.seq_len + 1, self.stride):
                self.window_indices.append((s_idx, start_idx))

    def _resolve_file_paths(self, source: Union[str, List[str]]) -> List[str]:
        if isinstance(source, list):
            return source
        if os.path.isdir(source):
            return sorted(glob.glob(os.path.join(source, '*.npz')) + glob.glob(os.path.join(source, '*.csv')))
        if os.path.isfile(source):
            return [source]
        # Glob pattern match
        matches = sorted(glob.glob(source))
        return matches

    def _load_session(self, path: str) -> Optional[Dict[str, np.ndarray]]:
        if path.endswith('.npz'):
            data = np.load(path, allow_pickle=True)
            # Check required keys
            if 'features' in data and 'gt_velocity' in data:
                return {
                    'features': data['features'].astype(np.float32),
                    'gt_velocity': data['gt_velocity'].astype(np.float32),
                    'timestamps': data.get('timestamps', np.arange(len(data['features']))).astype(np.float64)
                }
            elif 'imu' in data and 'radar_summary' in data:
                # Assemble from raw topic components
                radar = data['radar_summary']     # (N, 5)
                lidar = data['lidar_summary']     # (N, 5)
                imu = data['imu']                 # (N, 6)
                wheel = data['wheel_odometry']    # (N, 3)
                gt_vel = data.get('gt_velocity', np.zeros((len(radar), 3), dtype=np.float32))

                # Compute derived features
                acc_mag = np.linalg.norm(imu[:, 0:3], axis=1, keepdims=True)
                acc_var = np.var(imu[:, 0:3], axis=1, keepdims=True)
                gyr_mag = np.linalg.norm(imu[:, 3:6], axis=1, keepdims=True)

                features = np.hstack([
                    radar,
                    lidar,
                    acc_mag, acc_var, gyr_mag,
                    wheel
                ]).astype(np.float32)

                return {
                    'features': features,
                    'gt_velocity': gt_vel.astype(np.float32),
                    'timestamps': data.get('timestamps', np.arange(len(features))).astype(np.float64)
                }

        elif path.endswith('.csv'):
            import pandas as pd
            df = pd.read_csv(path)
            # Find feature columns matching names or fallback to first 16 numeric columns
            matching = [col for col in self.FEATURE_NAMES if col in df.columns]
            if len(matching) == len(self.FEATURE_NAMES):
                feats = df[self.FEATURE_NAMES].values.astype(np.float32)
            else:
                num_cols = df.select_dtypes(include=[np.number]).columns
                feats = df[num_cols[:16]].values.astype(np.float32)

            gt_cols = [c for c in ['gt_vx', 'gt_vy', 'gt_vz', 'vel_x', 'vel_y', 'vel_z'] if c in df.columns]
            if len(gt_cols) >= 3:
                gt_vel = df[gt_cols[:3]].values.astype(np.float32)
            else:
                gt_vel = np.zeros((len(feats), 3), dtype=np.float32)

            return {'features': feats, 'gt_velocity': gt_vel, 'timestamps': np.arange(len(feats))}

        return None

    def export_norm_params(self, save_path: str):
        """Export normalization parameters to JSON file for deployment."""
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        data = {
            'mean': self.mean.tolist(),
            'std': self.std.tolist(),
            'feature_names': self.FEATURE_NAMES
        }
        with open(save_path, 'w') as f:
            json.dump(data, f, indent=2)

    def __len__(self) -> int:
        return len(self.window_indices)

    def __getitem__(self, idx: int):
        s_idx, start = self.window_indices[idx]
        session = self.sessions[s_idx]

        feats = session['features'][start : start + self.seq_len].copy()
        gt_vel = session['gt_velocity'][start + self.seq_len - 1].copy()

        # Synthetic atmospheric degradation augmentation
        if self.augment_degradation and np.random.rand() > 0.5:
            # Simulate fog/dust: attenuate lidar, increase backscatter
            fog_factor = np.random.uniform(0.3, 1.0)
            feats[:, 5] *= (1.0 - 0.7 * fog_factor)   # LiDAR point count drops
            feats[:, 8] += (0.6 * fog_factor)         # Backscatter ratio spikes
            # Radar stays robust (RF penetration), slightly increases reflection dispersion
            feats[:, 4] += np.random.normal(0, 0.05, size=self.seq_len)

        # Normalize
        if self.normalize:
            feats = (feats - self.mean) / self.std

        if HAS_TORCH:
            return torch.from_numpy(feats), torch.from_numpy(gt_vel)
        return feats, gt_vel
