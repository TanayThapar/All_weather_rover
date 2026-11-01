"""
4D mmWave Radar Kinematics and Doppler Measurement Engine.

Implements:
1. Exact radial Doppler ego-velocity projection for arbitrary 6-DOF sensor extrinsics.
2. Robust RANSAC-based static target extraction and dynamic object rejection.
3. Analytical measurement Jacobians for Error-State Kalman Filter (ES-EKF).
"""

from typing import Tuple, List, Optional
import numpy as np
from .state_vector import skew_symmetric, RoverState


class RadarTarget:
    """Individual 4D radar point reflection."""
    __slots__ = ('x', 'y', 'z', 'doppler', 'snr', 'range_m', 'bearing')

    def __init__(self, x: float, y: float, z: float, doppler: float, snr: float = 1.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.doppler = float(doppler)
        self.snr = float(snr)
        self.range_m = float(np.sqrt(x**2 + y**2 + z**2))
        norm = max(self.range_m, 1e-6)
        self.bearing = np.array([x / norm, y / norm, z / norm], dtype=np.float64)


class RadarDopplerKinematics:
    """
    Kinematic solver and ES-EKF Jacobian generator for 4D FMCW mmWave Radar.
    Extrinsics: radar frame {r} relative to robot base_link {b}.
    """

    def __init__(self,
                 r_br: np.ndarray = None,
                 R_br: np.ndarray = None,
                 inlier_threshold_mps: float = 0.25,
                 min_inlier_ratio: float = 0.40):
        """
        Args:
            r_br: Lever arm [x, y, z] from base_link origin to radar antenna center.
            R_br: 3x3 rotation matrix from base_link to radar frame.
            inlier_threshold_mps: Velocity error threshold (m/s) for Doppler RANSAC.
            min_inlier_ratio: Minimum ratio of inliers required to accept instantaneous velocity.
        """
        # Default offset matching URDF sensors.xacro (x=0.40, y=0.0, z=0.15)
        self.r_br = np.array([0.40, 0.0, 0.15], dtype=np.float64) if r_br is None else np.asarray(r_br, dtype=np.float64)
        self.R_br = np.eye(3, dtype=np.float64) if R_br is None else np.asarray(R_br, dtype=np.float64)
        self.inlier_threshold_mps = inlier_threshold_mps
        self.min_inlier_ratio = min_inlier_ratio

    def project_doppler(self,
                        target: RadarTarget,
                        v_body: np.ndarray,
                        omega_body: np.ndarray) -> float:
        """
        Compute predicted radial Doppler velocity for a static target:
        v_r = R_br^T * (v_body + omega_body x r_br)
        v_doppler = - u_i^T * v_r
        """
        v_radar = self.R_br.T @ (v_body + np.cross(omega_body, self.r_br))
        return float(-np.dot(target.bearing, v_radar))

    def solve_ego_velocity_ransac(self,
                                  targets: List[RadarTarget],
                                  omega_body: np.ndarray,
                                  max_iterations: int = 40) -> Tuple[Optional[np.ndarray], List[RadarTarget]]:
        """
        Solve 3D translational radar velocity v_radar via Doppler RANSAC.
        Filters out moving dynamic targets (pedestrians, moving foliage, vehicles).

        Returns:
            v_body_estimated (3,) or None if insufficient inliers.
            inlier_targets: List of RadarTarget belonging to static background.
        """
        if len(targets) < 5:
            return None, []

        bearings = np.array([t.bearing for t in targets], dtype=np.float64)  # (N, 3)
        dopplers = np.array([t.doppler for t in targets], dtype=np.float64)  # (N,)

        # Linear system: - u_i^T * v_radar = doppler_i => u_i^T * v_radar = - doppler_i
        best_inliers = []
        best_v_radar = None

        N = len(targets)
        for _ in range(max_iterations):
            idx = np.random.choice(N, 3, replace=False)
            A_sample = bearings[idx]
            b_sample = -dopplers[idx]

            try:
                v_cand = np.linalg.lstsq(A_sample, b_sample, rcond=None)[0]
            except np.linalg.LinAlgError:
                continue

            # Evaluate residual on all targets
            pred = -bearings @ v_cand
            res = np.abs(pred - dopplers)
            inliers = np.where(res < self.inlier_threshold_mps)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_v_radar = v_cand

        if len(best_inliers) < max(4, int(N * self.min_inlier_ratio)):
            return None, []

        # Refine on all inliers
        A_inliers = bearings[best_inliers]
        b_inliers = -dopplers[best_inliers]
        v_radar_refined = np.linalg.lstsq(A_inliers, b_inliers, rcond=None)[0]

        # Convert v_radar -> v_body:
        # v_radar = R_br^T * (v_body + omega x r_br) => v_body = R_br * v_radar - omega x r_br
        v_body_est = self.R_br @ v_radar_refined - np.cross(omega_body, self.r_br)
        inlier_targets = [targets[i] for i in best_inliers]

        return v_body_est, inlier_targets

    def compute_doppler_jacobian(self,
                                 state: RoverState,
                                 omega_meas: np.ndarray,
                                 target: RadarTarget) -> np.ndarray:
        """
        Compute 1x15 Error-State Jacobian H_i = d(v_D) / d(delta_x)
        State order: [delta_p(3), delta_v(3), delta_theta(3), delta_b_a(3), delta_b_g(3)]
        """
        H = np.zeros(15, dtype=np.float64)
        u_i = target.bearing  # (3,)
        R_wb = state.rot_matrix
        R_bw = R_wb.T

        # v_body = R_bw * v_world
        # v_radar = R_br^T * (R_bw * v_world + (omega_meas - b_g) x r_br)
        # v_D = - u_i^T * v_radar

        # 1. d(v_D) / d(delta_v_world) = - u_i^T * R_br^T * R_bw
        d_vel = - (u_i @ self.R_br.T) @ R_bw
        H[3:6] = d_vel

        # 2. d(v_D) / d(delta_theta)
        # Rotation perturbation: R_wb -> R_wb * (I + [delta_theta]x) => R_bw -> (I - [delta_theta]x) * R_bw
        # d(R_bw * v_world) = [R_bw * v_world]x * delta_theta
        v_body = R_bw @ state.v
        d_theta = (u_i @ self.R_br.T) @ skew_symmetric(v_body)
        H[6:9] = d_theta

        # 3. d(v_D) / d(delta_b_g)
        # omega_true = omega_meas - b_g - delta_b_g
        # d(v_D) / d(delta_b_g) = - u_i^T * R_br^T * [r_br]x
        d_bg = - (u_i @ self.R_br.T) @ skew_symmetric(self.r_br)
        H[12:15] = d_bg

        return H
