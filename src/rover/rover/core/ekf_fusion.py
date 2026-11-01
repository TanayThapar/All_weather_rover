"""
Degradation-Aware Error-State Extended Kalman Filter (ES-EKF).
Fuses 6-DOF IMU, 4D mmWave Radar Doppler, LiDAR odometry, Wheel encoders, and GPS.
Features dynamic uncertainty scaling driven by real-time environmental degradation estimation.
"""

from typing import List, Optional, Tuple
import numpy as np

from .state_vector import (
    RoverState,
    skew_symmetric,
    quat_to_rot_matrix,
    quat_multiply,
    axis_angle_to_quat,
    quat_normalize
)
from .doppler_kinematics import RadarTarget, RadarDopplerKinematics


class DegradationAwareEKF:
    """
    Error-State Extended Kalman Filter with adaptive covariance scaling.
    """

    def __init__(self,
                 gravity: float = 9.80665,
                 radar_kinematics: Optional[RadarDopplerKinematics] = None):
        self.g_w = np.array([0.0, 0.0, -gravity], dtype=np.float64)
        self.state = RoverState()
        self.radar_kine = radar_kinematics or RadarDopplerKinematics()

        # 15x15 Error State Covariance
        self.P = np.eye(15, dtype=np.float64) * 0.01
        self.P[0:3, 0:3] = np.eye(3) * 0.1     # Position uncertainty (m^2)
        self.P[3:6, 3:6] = np.eye(3) * 1.0     # Velocity uncertainty (m/s)^2
        self.P[6:9, 6:9] = np.eye(3) * 0.05    # Orientation uncertainty (rad^2)
        self.P[9:12, 9:12] = np.eye(3) * 1e-3  # Accel bias uncertainty
        self.P[12:15, 12:15] = np.eye(3) * 1e-4# Gyro bias uncertainty

        # Continuous-time process noise spectral densities
        self.accel_noise_density = 0.08      # m/s^2 / sqrt(Hz)
        self.gyro_noise_density = 0.005      # rad/s / sqrt(Hz)
        self.accel_bias_random_walk = 0.001  # m/s^3 / sqrt(Hz)
        self.gyro_bias_random_walk = 0.0001  # rad/s^2 / sqrt(Hz)

        # Baseline measurement noise standard deviations
        self.base_sigma_radar_doppler = 0.15 # m/s
        self.base_sigma_lidar_pos = 0.05     # m
        self.base_sigma_lidar_rot = 0.02     # rad
        self.base_sigma_wheel_vel = 0.08     # m/s
        self.base_sigma_gps_pos = 0.40       # m

        # Current sensor reliability weights [0, 1] (1 = optimal, 0 = degraded)
        self.sensor_weights = {
            'radar': 1.0,
            'lidar': 1.0,
            'camera': 1.0,
            'wheel': 1.0,
            'gps': 1.0
        }

    def predict(self, acc_meas: np.ndarray, gyr_meas: np.ndarray, dt: float):
        """
        Inertial propagation using continuous IMU measurements.
        acc_meas: (3,) in Body frame
        gyr_meas: (3,) in Body frame
        dt: time step in seconds
        """
        if dt <= 0.0 or dt > 1.0:
            return

        acc_meas = np.asarray(acc_meas, dtype=np.float64)
        gyr_meas = np.asarray(gyr_meas, dtype=np.float64)

        # Unbiased specific force and angular velocity
        f_b = acc_meas - self.state.b_a
        omega_b = gyr_meas - self.state.b_g

        R_wb = self.state.rot_matrix
        f_w = R_wb @ f_b + self.g_w

        # 1. Propagate Nominal State
        p_new = self.state.p + self.state.v * dt + 0.5 * f_w * (dt ** 2)
        v_new = self.state.v + f_w * dt

        # Quaternion integration
        dtheta = omega_b * dt
        dq = axis_angle_to_quat(dtheta)
        q_new = quat_normalize(quat_multiply(self.state.q, dq))

        self.state.p = p_new
        self.state.v = v_new
        self.state.q = q_new

        # 2. Continuous-time error state Jacobian F_c (15x15)
        F = np.eye(15, dtype=np.float64)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R_wb @ skew_symmetric(f_b) * dt
        F[3:6, 9:12] = -R_wb * dt
        F[6:9, 6:9] = np.eye(3) - skew_symmetric(omega_b) * dt
        F[6:9, 12:15] = -np.eye(3) * dt

        # 3. Discrete Process Noise Q_d (15x15)
        Q = np.zeros((15, 15), dtype=np.float64)
        var_a = (self.accel_noise_density ** 2) * dt
        var_g = (self.gyro_noise_density ** 2) * dt
        var_ba = (self.accel_bias_random_walk ** 2) * dt
        var_bg = (self.gyro_bias_random_walk ** 2) * dt

        Q[3:6, 3:6] = np.eye(3) * var_a
        Q[6:9, 6:9] = np.eye(3) * var_g
        Q[9:12, 9:12] = np.eye(3) * var_ba
        Q[12:15, 12:15] = np.eye(3) * var_bg

        # 4. Propagate Covariance
        self.P = F @ self.P @ F.T + Q
        # Enforce symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update_radar_doppler(self,
                             targets: List[RadarTarget],
                             gyr_meas: np.ndarray,
                             noise_scale: float = 1.0) -> int:
        """
        Sequential measurement update for 4D Radar Doppler returns.
        Uses RANSAC to discard dynamic targets and updates the filter with static inliers.

        Returns:
            Number of inliers used in update.
        """
        if len(targets) == 0:
            return 0

        omega_b = gyr_meas - self.state.b_g
        _, inliers = self.radar_kine.solve_ego_velocity_ransac(targets, omega_b)
        if len(inliers) == 0:
            return 0

        sigma_d = self.base_sigma_radar_doppler * max(0.2, noise_scale)
        var_d = sigma_d ** 2

        # Subsample to at most 16 well-spread inliers for computational efficiency
        step = max(1, len(inliers) // 16)
        selected_inliers = inliers[::step]

        for target in selected_inliers:
            # Predicted Doppler
            v_pred = self.radar_kine.project_doppler(target, self.state.body_velocity, omega_b)
            residual = target.doppler - v_pred

            # Innovation gating (Chi-Square 3-sigma outlier rejection)
            H = self.radar_kine.compute_doppler_jacobian(self.state, gyr_meas, target)
            S = float(H @ self.P @ H.T + var_d)
            if S <= 0.0:
                continue

            # Standardized residual (Chi-square gating)
            if (residual ** 2) / S > 25.0:
                continue

            # Kalman gain K (15, 1)
            K = (self.P @ H.T) / S
            delta_x = K * residual

            # Error state injection and reset
            self.state.inject_error(delta_x)
            # Joseph form or standard update for stability
            I_KH = np.eye(15) - np.outer(K, H)
            self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * var_d

        self.P = 0.5 * (self.P + self.P.T)
        return len(selected_inliers)

    def update_lidar_odometry(self,
                              pos_meas: np.ndarray,
                              quat_meas: np.ndarray,
                              noise_scale: float = 1.0):
        """
        Update using LiDAR geometric scan matching pose.
        Under environmental degradation (fog/dust), noise_scale increases to de-weight
        corrupted geometric returns and prevent false aerosol wall pulling.
        """
        noise_scale = float(np.clip(noise_scale, 0.2, 50.0))
        pos_meas = np.asarray(pos_meas, dtype=np.float64)
        quat_meas = quat_normalize(np.asarray(quat_meas, dtype=np.float64))

        sigma_p = self.base_sigma_lidar_pos * noise_scale
        sigma_r = self.base_sigma_lidar_rot * noise_scale

        # 6-dim measurement residual [pos_error (3), rot_error (3)]
        r_p = pos_meas - self.state.p

        # Innovation gating: reject massive jumps caused by dense fog reflection walls
        if np.linalg.norm(r_p) > 4.0 * max(1.0, sigma_p):
            return

        # Orientation residual delta_q = q_meas * q_state^-1
        q_inv = np.array([self.state.q[0], -self.state.q[1], -self.state.q[2], -self.state.q[3]])
        dq = quat_multiply(quat_meas, q_inv)
        r_rot = 2.0 * dq[1:4]  # small angle approximation

        residual = np.concatenate([r_p, r_rot])

        # Measurement Jacobian H (6x15)
        H = np.zeros((6, 15), dtype=np.float64)
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 6:9] = np.eye(3)

        # Measurement covariance R (6x6)
        sigma_p = self.base_sigma_lidar_pos * noise_scale
        sigma_r = self.base_sigma_lidar_rot * noise_scale
        R = np.diag([sigma_p**2, sigma_p**2, sigma_p**2, sigma_r**2, sigma_r**2, sigma_r**2])

        # Standard Kalman Update
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        delta_x = K @ residual
        self.state.inject_error(delta_x)

        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_wheel_odometry(self,
                              v_linear_x: float,
                              omega_z: float,
                              noise_scale: float = 1.0):
        """
        Update using vehicle wheel encoders and non-holonomic ground constraint (vy=0, vz=0).
        """
        R_wb = self.state.rot_matrix
        v_body_meas = np.array([v_linear_x, 0.0, 0.0], dtype=np.float64)

        # Residual: v_body_meas - R_bw * v_world
        v_body_pred = R_wb.T @ self.state.v
        residual = v_body_meas - v_body_pred

        # Jacobian d(residual) / d(delta_x)
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = R_wb.T
        H[:, 6:9] = -R_wb.T @ skew_symmetric(self.state.v)

        sigma_v = self.base_sigma_wheel_vel * noise_scale
        # Non-holonomic lateral/vertical constraints are very stiff
        R = np.diag([sigma_v**2, 0.04**2, 0.04**2])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        delta_x = K @ residual
        self.state.inject_error(delta_x)

        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_learned_velocity_prior(self,
                                      v_body_pred: np.ndarray,
                                      sigma_vector: np.ndarray):
        """
        Fuse direct velocity prediction from the trained neural network model.
        v_body_pred: (3,) predicted body velocity
        sigma_vector: (3,) predicted standard deviation uncertainty
        """
        R_wb = self.state.rot_matrix
        v_pred = R_wb.T @ self.state.v
        residual = np.asarray(v_body_pred, dtype=np.float64) - v_pred

        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = R_wb.T
        H[:, 6:9] = -R_wb.T @ skew_symmetric(self.state.v)

        R = np.diag(np.clip(np.asarray(sigma_vector, dtype=np.float64) ** 2, 1e-4, 100.0))

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        delta_x = K @ residual
        self.state.inject_error(delta_x)

        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
