"""
CLI Script to train the Degradation-Aware Network (DA-Net) on personal recordings.

Usage:
    python3 -m rover.scripts.train --data_dir /path/to/recordings --epochs 40
    python3 -m rover.scripts.train --demo  # Generates synthetic sample and runs training test
"""

import os
import argparse
import numpy as np

from ..learning.model import DegradationCovarianceNet
from ..learning.dataset import RoverPersonalDataset
from ..learning.trainer import RoverTrainer


def generate_synthetic_session(filepath: str, n_steps: int = 1500):
    """Generate a realistic synthetic session with clear -> fog -> dust weather transition."""
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    t = np.linspace(0, 75, n_steps)
    dt = t[1] - t[0]

    # Motion: forward speed with turning maneuvers
    vx_body = 1.2 + 0.3 * np.sin(0.2 * t)
    omega_z = 0.15 * np.cos(0.2 * t)  # yaw rate (rad/s)
    theta = (0.15 / 0.2) * np.sin(0.2 * t)  # heading angle (rad)

    # 6-DOF ground truth orientation quaternion [w, x, y, z]
    qw = np.cos(0.5 * theta)
    qz = np.sin(0.5 * theta)
    gt_quat = np.column_stack([qw, np.zeros_like(theta), np.zeros_like(theta), qz])

    # World frame velocities
    vx_world = vx_body * np.cos(theta)
    vy_world = vx_body * np.sin(theta)
    gt_velocity = np.column_stack([vx_world, vy_world, np.zeros_like(vx_body)])

    # Specific force in Body frame (including centripetal acceleration and gravity)
    ax_body = np.gradient(vx_body, dt) + np.random.normal(0, 0.02, n_steps)
    ay_body = (vx_body * omega_z) + np.random.normal(0, 0.01, n_steps)
    az_body = 9.80665 + np.random.normal(0, 0.02, n_steps)
    imu_accel = np.column_stack([ax_body, ay_body, az_body])
    imu_gyro = np.column_stack([np.zeros_like(t), np.zeros_like(t), omega_z + np.random.normal(0, 0.005, n_steps)])
    imu_raw = np.hstack([imu_accel, imu_gyro])

    acc_mag = np.linalg.norm(imu_accel, axis=1, keepdims=True)
    acc_var = np.full((n_steps, 1), 0.01)
    gyr_mag = np.abs(omega_z).reshape(-1, 1)

    # Weather stages:
    # 0 -> 33%: Clear
    # 33% -> 66%: Dense fog (LiDAR backscatter rises, points drop)
    # 66% -> 100%: Dust storm (LiDAR blinds, radar Doppler remains strong)
    i1 = int(n_steps * 0.33)
    i2 = int(n_steps * 0.66)
    fog_intensity = np.zeros(n_steps)
    fog_intensity[i1:i2] = np.linspace(0.1, 0.9, i2 - i1)
    fog_intensity[i2:] = 0.95

    # LiDAR
    lidar_pts = (900 * (1.0 - 0.8 * fog_intensity) + np.random.normal(0, 20, n_steps)).reshape(-1, 1)
    lidar_pts = np.clip(lidar_pts, 30, 1000)
    lidar_range = (18.0 * (1.0 - 0.7 * fog_intensity) + np.random.normal(0, 0.5, n_steps)).reshape(-1, 1)
    lidar_std = np.full((n_steps, 1), 0.3)
    lidar_backscatter = (0.02 + 0.85 * fog_intensity + np.random.normal(0, 0.02, n_steps)).reshape(-1, 1)
    lidar_entropy = (0.8 - 0.5 * fog_intensity).reshape(-1, 1)
    lidar_summary = np.hstack([lidar_pts, lidar_range, lidar_std, lidar_backscatter, lidar_entropy])

    # 4D Radar (RF penetrates fog/dust!)
    radar_count = np.random.randint(18, 45, (n_steps, 1)).astype(float)
    radar_mean_dop = (-vx_body + np.random.normal(0, 0.08, n_steps)).reshape(-1, 1)
    radar_std_dop = np.full((n_steps, 1), 0.15)
    radar_snr = (18.0 - 2.0 * fog_intensity + np.random.normal(0, 0.5, n_steps)).reshape(-1, 1)
    radar_disp = np.full((n_steps, 1), 0.4)
    radar_summary = np.hstack([radar_count, radar_mean_dop, radar_std_dop, radar_snr, radar_disp])

    # Wheel odometry
    wheel_vx = (vx_body + np.random.normal(0, 0.03, n_steps)).reshape(-1, 1)
    wheel_wz = (omega_z + np.random.normal(0, 0.01, n_steps)).reshape(-1, 1)
    wheel_steer = (0.15 * np.cos(0.2 * t)).reshape(-1, 1)
    wheel_odometry = np.hstack([wheel_vx, wheel_wz, wheel_steer])

    features = np.hstack([
        radar_summary,
        lidar_summary,
        acc_mag, acc_var, gyr_mag,
        wheel_odometry
    ]).astype(np.float32)

    np.savez_compressed(
        filepath,
        features=features,
        imu=imu_raw.astype(np.float32),
        gt_quat=gt_quat.astype(np.float32),
        gt_velocity=gt_velocity.astype(np.float32),
        timestamps=t
    )
    print(f"Generated synthetic training recording: {filepath} ({n_steps} steps)")


def main():
    parser = argparse.ArgumentParser(description="Train DA-Net on personal recordings")
    parser.add_argument('--data_dir', type=str, default='data/recordings',
                        help="Path to folder containing .npz/.csv session files")
    parser.add_argument('--val_dir', type=str, default=None,
                        help="Optional separate validation folder")
    parser.add_argument('--epochs', type=int, default=30, help="Number of training epochs")
    parser.add_argument('--batch_size', type=int, default=32, help="Mini-batch size")
    parser.add_argument('--lr', type=float, default=1e-3, help="Learning rate")
    parser.add_argument('--checkpoint_dir', type=str, default='checkpoints', help="Output checkpoint directory")
    parser.add_argument('--demo', action='store_true', help="Run demo with automatically generated synthetic dataset")

    args = parser.parse_args()

    if args.demo:
        demo_dir = os.path.join(args.checkpoint_dir, 'demo_data')
        train_file = os.path.join(demo_dir, 'session_train.npz')
        val_file = os.path.join(demo_dir, 'session_val.npz')
        generate_synthetic_session(train_file, n_steps=2000)
        generate_synthetic_session(val_file, n_steps=600)
        args.data_dir = train_file
        args.val_dir = val_file

    print(f"📂 Loading dataset from: {args.data_dir}")
    train_ds = RoverPersonalDataset(args.data_dir, seq_len=20, stride=2, normalize=True)

    val_ds = None
    if args.val_dir:
        val_ds = RoverPersonalDataset(
            args.val_dir,
            seq_len=20,
            stride=2,
            normalize=True,
            norm_params={'mean': train_ds.mean, 'std': train_ds.std}
        )

    model = DegradationCovarianceNet(in_features=16, hidden_dim=64, num_layers=2)
    trainer = RoverTrainer(
        model=model,
        train_dataset=train_ds,
        val_dataset=val_ds,
        batch_size=args.batch_size,
        lr=args.lr,
        checkpoint_dir=args.checkpoint_dir
    )

    trainer.fit(epochs=args.epochs)


if __name__ == '__main__':
    main()
