"""
ROS 2 Degradation-Aware State Estimation & Multi-Sensor Fusion Node.

Subscribes:
  - /imu                     (sensor_msgs/Imu)
  - /radar/scan              (sensor_msgs/PointCloud2)
  - /odom                    (nav_msgs/Odometry)
  - /lidar/odom              (nav_msgs/Odometry)
  - /gps/fix                 (sensor_msgs/NavSatFix)

Publishes:
  - /rover/filtered_odom     (nav_msgs/Odometry)
  - /tf                      (odom -> base_link transform)
"""

import os
import json
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu, PointCloud2, NavSatFix
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    import sensor_msgs_py.point_cloud2 as pc2
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

from ..core.ekf_fusion import DegradationAwareEKF
from ..core.doppler_kinematics import RadarTarget, RadarDopplerKinematics
from ..learning.model import DegradationCovarianceNet, NumPyDegradationPredictor


class RoverFusionNode(Node):
    """Real-time ROS 2 node for degradation-aware radar-inertial state estimation."""

    def __init__(self):
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy) is not installed in the current environment.")

        super().__init__('rover_fusion_node')

        # Declare ROS 2 parameters
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('norm_params_path', '')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('radar_x_offset', 0.40)
        self.declare_parameter('radar_z_offset', 0.15)

        ckpt_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        norm_path = self.get_parameter('norm_params_path').get_parameter_value().string_value
        radar_x = self.get_parameter('radar_x_offset').get_parameter_value().double_value
        radar_z = self.get_parameter('radar_z_offset').get_parameter_value().double_value

        # Initialize EKF and Kinematics
        radar_kine = RadarDopplerKinematics(r_br=np.array([radar_x, 0.0, radar_z]))
        self.ekf = DegradationAwareEKF(radar_kinematics=radar_kine)

        # Initialize Learned Predictor or Heuristic Fallback
        self.predictor = None
        self.norm_mean = np.zeros(16, dtype=np.float32)
        self.norm_std = np.ones(16, dtype=np.float32)

        if norm_path and os.path.exists(norm_path):
            with open(norm_path, 'r') as f:
                nd = json.load(f)
                self.norm_mean = np.array(nd['mean'], dtype=np.float32)
                self.norm_std = np.array(nd['std'], dtype=np.float32)
            self.get_logger().info(f"Loaded normalization parameters from {norm_path}")

        if ckpt_path and os.path.exists(ckpt_path) and HAS_TORCH:
            self.get_logger().info(f"Loading trained DA-Net weights from: {ckpt_path}")
            ckpt = torch.load(ckpt_path, map_location='cpu')
            model = DegradationCovarianceNet(in_features=16, hidden_dim=64, num_layers=2)
            model.load_state_dict(ckpt['model_state_dict'])
            model.eval()
            self.predictor = model
        else:
            self.get_logger().info("Using pure NumPy degradation estimator.")
            self.predictor = NumPyDegradationPredictor()

        # Telemetry sliding buffer for neural network input (16 features)
        self.feature_history = np.zeros((20, 16), dtype=np.float32)
        self.last_imu_stamp = None
        self.latest_gyro = np.zeros(3)

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, sensor_qos)
        self.sub_radar = self.create_subscription(PointCloud2, '/radar/scan', self.radar_callback, sensor_qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_lidar = self.create_subscription(Odometry, '/lidar/odom', self.lidar_callback, 10)

        # Publishers
        self.pub_filtered_odom = self.create_publisher(Odometry, '/rover/filtered_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("🚜 All Weather Rover State Estimation Node Online.")

    def imu_callback(self, msg: Imu):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_stamp is None:
            self.last_imu_stamp = curr_time
            return

        dt = curr_time - self.last_imu_stamp
        self.last_imu_stamp = curr_time

        if dt <= 0.0 or dt > 0.5:
            return

        acc_meas = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float64)

        gyr_meas = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float64)
        self.latest_gyro = gyr_meas

        # 1. EKF Propagation
        self.ekf.predict(acc_meas, gyr_meas, dt)

        # 2. Update IMU feature vector
        acc_mag = float(np.linalg.norm(acc_meas))
        gyr_mag = float(np.linalg.norm(gyr_meas))
        self.feature_history[-1, 10] = acc_mag
        self.feature_history[-1, 12] = gyr_mag

        # 3. Publish Filtered Odometry & TF
        self.publish_odometry(msg.header.stamp)

    def radar_callback(self, msg: PointCloud2):
        targets = []
        try:
            # Read point fields: x, y, z, and doppler / velocity
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'doppler'), skip_nans=True):
                targets.append(RadarTarget(x=p[0], y=p[1], z=p[2], doppler=p[3]))
        except Exception:
            return

        if not targets:
            return

        # Infer dynamic degradation weight
        cov_scale = self._predict_covariance_scale()
        scale_radar = float(cov_scale[0])

        # EKF Doppler update
        self.ekf.update_radar_doppler(targets, self.latest_gyro, noise_scale=scale_radar)

        # Record radar features
        dops = [t.doppler for t in targets]
        self.feature_history[-1, 0] = len(targets)
        self.feature_history[-1, 1] = float(np.mean(dops))
        self.feature_history[-1, 2] = float(np.std(dops))

    def odom_callback(self, msg: Odometry):
        v_forward = msg.twist.twist.linear.x
        omega_z = msg.twist.twist.angular.z

        cov_scale = self._predict_covariance_scale()
        self.ekf.update_wheel_odometry(v_forward, omega_z, noise_scale=float(cov_scale[2]))

        self.feature_history[-1, 13] = v_forward
        self.feature_history[-1, 14] = omega_z

    def lidar_callback(self, msg: Odometry):
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        cov_scale = self._predict_covariance_scale()
        scale_lidar = float(cov_scale[1])

        # Update LiDAR pose with dynamic noise scaling
        self.ekf.update_lidar_odometry(pos, quat, noise_scale=scale_lidar)

    def _predict_covariance_scale(self) -> np.ndarray:
        """Query neural network or fallback for covariance scale factors."""
        if HAS_TORCH and isinstance(self.predictor, torch.nn.Module):
            feat_norm = (self.feature_history - self.norm_mean) / self.norm_std
            with torch.no_grad():
                t_in = torch.from_numpy(feat_norm).unsqueeze(0).float()
                _, cov_scale, _, _ = self.predictor(t_in)
                return cov_scale.squeeze(0).cpu().numpy()
        else:
            _, cov_scale, _, _ = self.predictor.predict(self.feature_history[-1])
            return cov_scale

    def publish_odometry(self, stamp):
        state = self.ekf.state

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Pose
        odom_msg.pose.pose.position.x = float(state.p[0])
        odom_msg.pose.pose.position.y = float(state.p[1])
        odom_msg.pose.pose.position.z = float(state.p[2])
        odom_msg.pose.pose.orientation.w = float(state.q[0])
        odom_msg.pose.pose.orientation.x = float(state.q[1])
        odom_msg.pose.pose.orientation.y = float(state.q[2])
        odom_msg.pose.pose.orientation.z = float(state.q[3])

        # Twist
        odom_msg.twist.twist.linear.x = float(state.v[0])
        odom_msg.twist.twist.linear.y = float(state.v[1])
        odom_msg.twist.twist.linear.z = float(state.v[2])

        self.pub_filtered_odom.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = state.p[0]
        t.transform.translation.y = state.p[1]
        t.transform.translation.z = state.p[2]
        t.transform.rotation.w = state.q[0]
        t.transform.rotation.x = state.q[1]
        t.transform.rotation.y = state.q[2]
        t.transform.rotation.z = state.q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    if not HAS_ROS2:
        print("ROS 2 (rclpy) is not available. Source ROS 2 environment first.")
        return
    rclpy.init(args=args)
    node = RoverFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
