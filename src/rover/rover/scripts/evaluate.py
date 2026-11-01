"""
Trajectory Evaluation and Benchmark Engine.
Computes:
1. Absolute Trajectory Error (ATE) RMSE.
2. Relative Pose Error (RPE) translational and rotational drift.
3. Degradation resilience: compares proposed DA-EKF against Unweighted EKF and Pure Odometry.
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

from ..core.state_vector import RoverState
from ..core.ekf_fusion import DegradationAwareEKF
from ..core.doppler_kinematics import RadarTarget
from ..learning.model import DegradationCovarianceNet, NumPyDegradationPredictor
from ..learning.dataset import RoverPersonalDataset

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


def compute_ate_rmse(gt_pos: np.ndarray, est_pos: np.ndarray) -> float:
    """Compute Absolute Trajectory Error (ATE) RMSE in meters."""
    errors = np.linalg.norm(gt_pos - est_pos, axis=1)
    return float(np.sqrt(np.mean(errors ** 2)))


def compute_rpe_drift(gt_pos: np.ndarray, est_pos: np.ndarray, delta_steps: int = 10) -> float:
    """Compute Relative Pose Error (RPE) translation drift per meter."""
    gt_disp = gt_pos[delta_steps:] - gt_pos[:-delta_steps]
    est_disp = est_pos[delta_steps:] - est_pos[:-delta_steps]
    errors = np.linalg.norm(gt_disp - est_disp, axis=1)
    return float(np.mean(errors))


def run_evaluation(data_path: str,
                   checkpoint_path: Optional[str] = None,
                   plot: bool = True,
                   save_plot: Optional[str] = None):
    print(f"📊 Running evaluation on: {data_path}")
    data = np.load(data_path, allow_pickle=True)
    features = data['features']
    gt_velocity = data['gt_velocity']
    timestamps = data.get('timestamps', np.arange(len(features)) * 0.05)
    n_steps = len(features)

    # Reconstruct ground truth position by dead reckoning
    gt_pos = np.zeros((n_steps, 3))
    for i in range(1, n_steps):
        dt = timestamps[i] - timestamps[i - 1]
        gt_pos[i] = gt_pos[i - 1] + gt_velocity[i - 1] * dt

    # Load neural network predictor if checkpoint exists
    predictor = None
    if checkpoint_path and os.path.exists(checkpoint_path) and HAS_TORCH:
        print(f"   Loading trained model checkpoint: {checkpoint_path}")
        ckpt = torch.load(checkpoint_path, map_location='cpu')
        model = DegradationCovarianceNet(in_features=16, hidden_dim=64, num_layers=2)
        model.load_state_dict(ckpt['model_state_dict'])
        model.eval()
        predictor = model
    else:
        print("   Using heuristic degradation predictor (no PyTorch checkpoint provided).")
        predictor = NumPyDegradationPredictor()

    # Load normalization parameters if available
    norm_mean = np.zeros(16)
    norm_std = np.ones(16)
    norm_file = os.path.join(os.path.dirname(checkpoint_path) if checkpoint_path else '.', 'norm_params.json')
    if os.path.exists(norm_file):
        import json
        with open(norm_file, 'r') as f:
            nd = json.load(f)
            norm_mean = np.array(nd['mean'])
            norm_std = np.array(nd['std'])

    # Instantiate filters for comparative benchmark:
    # 1. Proposed: Degradation-Aware EKF (adaptive noise covariance)
    ekf_proposed = DegradationAwareEKF()
    # 2. Baseline: Standard Fixed-Covariance EKF (blind to fog/dust degradation)
    ekf_baseline = DegradationAwareEKF()

    pos_proposed = np.zeros((n_steps, 3))
    pos_baseline = np.zeros((n_steps, 3))
    pos_pure_odom = np.zeros((n_steps, 3))

    weight_history = []

    for i in range(1, n_steps):
        dt = float(timestamps[i] - timestamps[i - 1])
        if dt <= 0:
            dt = 0.05

        feat = features[i]
        # IMU specific force and gyro
        if 'imu' in data:
            acc_meas = data['imu'][i, 0:3]
            gyr_meas = data['imu'][i, 3:6]
        else:
            acc_meas = np.array([0.0, 0.0, 9.81])
            gyr_meas = np.array([0.0, 0.0, feat[14]])

        # 1. Predict
        ekf_proposed.predict(acc_meas, gyr_meas, dt)
        ekf_baseline.predict(acc_meas, gyr_meas, dt)

        # Predict degradation weights
        if HAS_TORCH and isinstance(predictor, torch.nn.Module):
            feat_norm = (feat - norm_mean) / norm_std
            with torch.no_grad():
                t_feat = torch.from_numpy(feat_norm).float().view(1, 1, 16)
                w, cov_scale, v_p, v_lv = predictor(t_feat)
                w = w.squeeze().cpu().numpy()
                cov_scale = cov_scale.squeeze().cpu().numpy()
        else:
            w, cov_scale, _, _ = predictor.predict(feat)

        weight_history.append(w)

        # 2. Radar Update (RF Doppler - immune to fog)
        radar_count = int(feat[0])
        mean_dop = float(feat[1])
        radar_targets = [
            RadarTarget(
                x=float(np.random.uniform(2.0, 25.0)),
                y=float(np.random.uniform(-8.0, 8.0)),
                z=0.1,
                doppler=mean_dop + float(np.random.normal(0, 0.05))
            )
            for _ in range(max(4, min(radar_count, 12)))
        ]
        # Proposed adapts covariance scale; baseline uses fixed scale = 1.0
        ekf_proposed.update_radar_doppler(radar_targets, gyr_meas, noise_scale=cov_scale[0])
        ekf_baseline.update_radar_doppler(radar_targets, gyr_meas, noise_scale=1.0)

        # 3. LiDAR Update (Geometric scan match corrupted by fog/dust aerosol backscatter)
        backscatter = max(0.0, float(feat[8]))
        lidar_corrupt_noise = np.random.normal(0, 0.05 + 1.2 * backscatter, size=3)
        lidar_pos_meas = gt_pos[i] + lidar_corrupt_noise

        if 'gt_quat' in data:
            lidar_quat_meas = data['gt_quat'][i]
        else:
            lidar_quat_meas = np.array([1.0, 0.0, 0.0, 0.0])

        ekf_proposed.update_lidar_odometry(lidar_pos_meas, lidar_quat_meas, noise_scale=cov_scale[1])
        # Baseline trusts degraded LiDAR blindly!
        ekf_baseline.update_lidar_odometry(lidar_pos_meas, lidar_quat_meas, noise_scale=1.0)

        # 4. Wheel odometry with traction slip during adverse weather (mud, wet terrain)
        wheel_vx = float(feat[13])
        wheel_slip = 1.0 + 0.35 * backscatter
        slipped_wheel_vx = float(wheel_vx * wheel_slip)
        ekf_proposed.update_wheel_odometry(slipped_wheel_vx, gyr_meas[2], noise_scale=cov_scale[2])
        ekf_baseline.update_wheel_odometry(slipped_wheel_vx, gyr_meas[2], noise_scale=1.0)

        pos_proposed[i] = ekf_proposed.state.p
        pos_baseline[i] = ekf_baseline.state.p

        # Pure wheel odometry dead reckoning
        pos_pure_odom[i] = pos_pure_odom[i - 1] + np.array([wheel_vx * dt, 0.0, 0.0])

    # Compute Benchmark Metrics
    ate_proposed = compute_ate_rmse(gt_pos, pos_proposed)
    ate_baseline = compute_ate_rmse(gt_pos, pos_baseline)
    ate_pure_odom = compute_ate_rmse(gt_pos, pos_pure_odom)

    rpe_proposed = compute_rpe_drift(gt_pos, pos_proposed)
    rpe_baseline = compute_rpe_drift(gt_pos, pos_baseline)

    print("\n" + "=" * 65)
    print("🏆 EXPERIMENTAL BENCHMARK RESULTS")
    print("=" * 65)
    print(f"{'Method':<35} | {'ATE RMSE (m)':<13} | {'RPE Drift (m)':<13}")
    print("-" * 65)
    print(f"{'Pure Wheel Dead Reckoning':<35} | {ate_pure_odom:<13.3f} | {'N/A':<13}")
    print(f"{'Standard Fixed-Covariance EKF':<35} | {ate_baseline:<13.3f} | {rpe_baseline:<13.3f}")
    print(f"{'Proposed DA-EKF (Degradation-Aware)':<35} | {ate_proposed:<13.3f} | {rpe_proposed:<13.3f}")
    print("=" * 65)
    improvement = ((ate_baseline - ate_proposed) / max(ate_baseline, 1e-4)) * 100.0
    print(f"🎯 Error reduction over standard baseline: {improvement:.1f}%\n")

    if plot or save_plot:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [2.5, 1]})

        # Trajectory
        ax1.plot(gt_pos[:, 0], gt_pos[:, 1], 'k--', label='Ground Truth (Reference)', linewidth=2.0)
        ax1.plot(pos_proposed[:, 0], pos_proposed[:, 1], 'g-', label='Proposed DA-EKF (Radar-Centric)', linewidth=2.0)
        ax1.plot(pos_baseline[:, 0], pos_baseline[:, 1], 'r:', label='Baseline Fixed EKF (LiDAR Blinded)', linewidth=1.5)
        ax1.set_title('Trajectory Estimation Under Aerosol Degradation (Fog/Dust Storm)', fontsize=12, fontweight='bold')
        ax1.set_xlabel('Global X (meters)')
        ax1.set_ylabel('Global Y (meters)')
        ax1.grid(True, linestyle='--', alpha=0.5)
        ax1.legend(loc='best')

        # Degradation confidence weights
        weights_arr = np.array(weight_history)
        t_arr = timestamps[1:]
        ax2.plot(t_arr, weights_arr[:, 0], 'b-', label='Radar Confidence (RF Penetration)')
        ax2.plot(t_arr, weights_arr[:, 1], 'r-', label='LiDAR Confidence (Aerosol Attenuation)')
        ax2.plot(t_arr, weights_arr[:, 3], 'gray', linestyle='--', label='Wheel Confidence')
        ax2.set_title('Dynamic Learned Sensor Reliability Weights', fontsize=11)
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Weight w_i in [0, 1]')
        ax2.set_ylim(-0.05, 1.05)
        ax2.grid(True, linestyle='--', alpha=0.5)
        ax2.legend(loc='lower left')

        plt.tight_layout()
        if save_plot:
            plt.savefig(save_plot, dpi=300)
            print(f"📈 Trajectory plot saved to: {save_plot}")
        if plot:
            plt.show()


def main():
    parser = argparse.ArgumentParser(description="Evaluate state estimation benchmark")
    parser.add_argument('--data', type=str, required=True, help="Path to evaluation .npz session")
    parser.add_argument('--checkpoint', type=str, default=None, help="Trained checkpoint .pt")
    parser.add_argument('--save_plot', type=str, default=None, help="Path to save evaluation figure PNG")
    parser.add_argument('--no_plot', action='store_true', help="Disable interactive plot display")

    args = parser.parse_args()
    run_evaluation(
        data_path=args.data,
        checkpoint_path=args.checkpoint,
        plot=not args.no_plot,
        save_plot=args.save_plot
    )


if __name__ == '__main__':
    main()
