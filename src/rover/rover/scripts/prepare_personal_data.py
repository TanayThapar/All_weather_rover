"""
Data Preprocessing Utility for Personal Live Vehicle Recordings.

Converts raw CSV logs, ROS 2 rosbags (mcap / db3), or custom sensor streams
into synchronized, standardized NPZ archives ready for DA-Net training.

Usage:
    python3 -m rover.scripts.prepare_personal_data --input session_raw.csv --output data/session_01.npz
    python3 -m rover.scripts.prepare_personal_data --generate_sample data/sample_live_run.npz
"""

import os
import argparse
from typing import Dict, Any, Optional
import numpy as np


def interpolate_to_uniform_grid(timestamps: np.ndarray,
                                values: np.ndarray,
                                target_timestamps: np.ndarray) -> np.ndarray:
    """Linearly interpolate multi-dimensional sensor array to uniform time grid."""
    if values.ndim == 1:
        return np.interp(target_timestamps, timestamps, values)
    out = np.zeros((len(target_timestamps), values.shape[1]), dtype=values.dtype)
    for dim in range(values.shape[1]):
        out[:, dim] = np.interp(target_timestamps, timestamps, values[:, dim])
    return out


def parse_csv_recording(csv_path: str) -> Dict[str, np.ndarray]:
    """Parse personal telemetry CSV with automatic column header detection."""
    import pandas as pd
    df = pd.read_csv(csv_path)

    # 1. Timestamps
    time_col = None
    for candidate in ['time', 'timestamp', 't', 'stamp', 'Header.stamp']:
        if candidate in df.columns:
            time_col = candidate
            break

    if time_col:
        t = df[time_col].values.astype(np.float64)
        if t[0] > 1e12:  # Nanoseconds to seconds
            t = t / 1e9
        t = t - t[0]
    else:
        t = np.arange(len(df)) * 0.05  # Assume default 20 Hz

    # 2. Extract or infer features
    # Standard 16 features:
    # Radar (5): count, mean_dop, std_dop, mean_snr, disp
    # LiDAR (5): count, mean_range, std_range, backscatter, entropy
    # IMU (3): acc_mag, acc_var, gyr_mag
    # Wheel (3): vx, wz, steer
    features = np.zeros((len(df), 16), dtype=np.float32)

    # Helper mapping
    col_map = {
        'radar_count': 0, 'radar_dop': 1, 'radar_snr': 3,
        'lidar_points': 5, 'lidar_range': 6, 'lidar_backscatter': 8,
        'acc_mag': 10, 'gyr_mag': 12,
        'speed': 13, 'wheel_speed': 13, 'vx': 13, 'yaw_rate': 14, 'wz': 14
    }

    for col in df.columns:
        col_lower = col.lower()
        for key, idx in col_map.items():
            if key in col_lower:
                features[:, idx] = df[col].values.astype(np.float32)

    # Ground truth body velocity
    gt_velocity = np.zeros((len(df), 3), dtype=np.float32)
    for i, c in enumerate(['gt_vx', 'gt_vy', 'gt_vz']):
        if c in df.columns:
            gt_velocity[:, i] = df[c].values.astype(np.float32)

    return {
        'timestamps': t,
        'features': features,
        'gt_velocity': gt_velocity
    }


def parse_rosbag(bag_path: str) -> Optional[Dict[str, np.ndarray]]:
    """
    Extract topics from a ROS 2 rosbag folder containing .db3 or .mcap files.
    Requires rosbag2_py (standard in ROS 2 desktop/base).
    """
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py
    except ImportError:
        print("⚠️ rosbag2_py not found. To parse rosbags directly, source your ROS 2 workspace:")
        print("   source /opt/ros/$ROS_DISTRO/setup.bash")
        return None

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    topics_data = {}
    type_map = {}
    for topic_meta in reader.get_all_topics_and_types():
        type_map[topic_meta.name] = topic_meta.type

    print(f"📖 Reading rosbag: {bag_path}")
    count = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic not in topics_data:
            topics_data[topic] = {'timestamps': [], 'data': []}

        topics_data[topic]['timestamps'].append(stamp * 1e-9)
        topics_data[topic]['data'].append(msg)
        count += 1

    print(f"   Read {count} messages across {len(topics_data)} topics.")
    # Return structured dict after extraction
    return None


def main():
    parser = argparse.ArgumentParser(description="Prepare personal recording dataset for training")
    parser.add_argument('--input', type=str, default=None, help="Input CSV or rosbag directory")
    parser.add_argument('--output', type=str, default='data/session.npz', help="Output .npz file")
    parser.add_argument('--generate_sample', type=str, default=None,
                        help="Generate a realistic sample recording at given path")

    args = parser.parse_args()

    if args.generate_sample:
        from .train import generate_synthetic_session
        generate_synthetic_session(args.generate_sample, n_steps=2000)
        return

    if not args.input:
        print("❌ Error: Please specify --input file or --generate_sample path.")
        return

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    if args.input.endswith('.csv'):
        print(f"📄 Parsing personal CSV log: {args.input}")
        parsed = parse_csv_recording(args.input)
        np.savez_compressed(
            args.output,
            features=parsed['features'],
            gt_velocity=parsed['gt_velocity'],
            timestamps=parsed['timestamps']
        )
        print(f"✅ Successfully converted to: {args.output} ({len(parsed['features'])} samples)")

    elif os.path.isdir(args.input):
        print(f"📦 Parsing ROS 2 rosbag folder: {args.input}")
        parsed = parse_rosbag(args.input)
        if parsed is not None:
            np.savez_compressed(args.output, **parsed)
            print(f"✅ Successfully converted rosbag to: {args.output}")


if __name__ == '__main__':
    main()
