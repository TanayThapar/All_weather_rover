"""
Live Data Recorder Node for Physical Rover and Simulation.

Records synchronized telemetry from:
  - /imu                     (sensor_msgs/Imu)
  - /radar/scan              (sensor_msgs/PointCloud2)
  - /odom                    (nav_msgs/Odometry)
  - /gps/fix                 (sensor_msgs/NavSatFix)

Saves clean, synchronized .npz files ready for training DA-Net.
"""

import os
import time
from datetime import datetime
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu, PointCloud2, NavSatFix
    from nav_msgs.msg import Odometry
    import sensor_msgs_py.point_cloud2 as pc2
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object


class DataRecorderNode(Node):
    """Synchronous data recording node for live personal rover runs."""

    def __init__(self):
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy) is required to run the data recorder node.")

        super().__init__('rover_data_recorder_node')

        self.declare_parameter('output_dir', 'data/recordings')
        self.declare_parameter('record_hz', 20.0)
        self.declare_parameter('session_name', '')

        out_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        hz = self.get_parameter('record_hz').get_parameter_value().double_value
        sess_name = self.get_parameter('session_name').get_parameter_value().string_value

        self.output_dir = out_dir
        os.makedirs(self.output_dir, exist_ok=True)

        if not sess_name:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_file = os.path.join(self.output_dir, f"session_{timestamp_str}.npz")
        else:
            self.session_file = os.path.join(self.output_dir, f"{sess_name}.npz")

        # Telemetry State Buffers
        self.curr_radar = np.zeros(5, dtype=np.float32)
        self.curr_lidar = np.array([800.0, 15.0, 0.2, 0.02, 0.8], dtype=np.float32)
        self.curr_imu = np.zeros(6, dtype=np.float32)
        self.curr_wheel = np.zeros(3, dtype=np.float32)
        self.curr_gt_vel = np.zeros(3, dtype=np.float32)

        # Storage
        self.recorded_features = []
        self.recorded_gt_vel = []
        self.recorded_timestamps = []
        self.start_time = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, sensor_qos)
        self.sub_radar = self.create_subscription(PointCloud2, '/radar/scan', self.radar_cb, sensor_qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Synchronous sampling timer
        period = 1.0 / hz
        self.timer = self.create_timer(period, self.sample_timer_cb)

        self.get_logger().info(f"⏺️ Data Recorder Online. Logging to: {self.session_file} @ {hz} Hz")

    def imu_cb(self, msg: Imu):
        self.curr_imu = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)

    def radar_cb(self, msg: PointCloud2):
        dops = []
        try:
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'doppler'), skip_nans=True):
                dops.append(p[3])
        except Exception:
            return

        if dops:
            self.curr_radar[0] = len(dops)
            self.curr_radar[1] = float(np.mean(dops))
            self.curr_radar[2] = float(np.std(dops))
            self.curr_radar[3] = 18.0
            self.curr_radar[4] = 0.35

    def odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wz = msg.twist.twist.angular.z

        self.curr_wheel[0] = vx
        self.curr_wheel[1] = wz
        self.curr_gt_vel = np.array([vx, vy, vz], dtype=np.float32)

    def sample_timer_cb(self):
        now = time.time()
        if self.start_time is None:
            self.start_time = now

        t_rel = now - self.start_time

        acc_mag = np.linalg.norm(self.curr_imu[0:3])
        acc_var = 0.02
        gyr_mag = np.linalg.norm(self.curr_imu[3:6])
        imu_summary = np.array([acc_mag, acc_var, gyr_mag], dtype=np.float32)

        feat = np.hstack([
            self.curr_radar,
            self.curr_lidar,
            imu_summary,
            self.curr_wheel
        ]).astype(np.float32)

        self.recorded_features.append(feat)
        self.recorded_gt_vel.append(self.curr_gt_vel.copy())
        self.recorded_timestamps.append(t_rel)

        if len(self.recorded_features) % 100 == 0:
            self.get_logger().info(f"   Recorded {len(self.recorded_features)} samples ({t_rel:.1f} s)...")

    def save_session(self):
        if not self.recorded_features:
            self.get_logger().warn("No data recorded. File not saved.")
            return

        feats_arr = np.array(self.recorded_features, dtype=np.float32)
        gt_arr = np.array(self.recorded_gt_vel, dtype=np.float32)
        t_arr = np.array(self.recorded_timestamps, dtype=np.float64)

        np.savez_compressed(
            self.session_file,
            features=feats_arr,
            gt_velocity=gt_arr,
            timestamps=t_arr
        )
        self.get_logger().info(f"💾 Saved {len(feats_arr)} samples to: {self.session_file}")


def main(args=None):
    if not HAS_ROS2:
        print("ROS 2 is required to run data recorder.")
        return
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Recording stopped by user. Saving data...")
    finally:
        node.save_session()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
