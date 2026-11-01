"""
State Vector and Kinematics Engine for Error-State Kalman Filter (ES-EKF).
Part of the Degradation-Aware 4D Radar-Inertial Fusion Pipeline.

Nominal State (16-dim):
    - p: Position (3) [x, y, z] in World frame
    - v: Velocity (3) [vx, vy, vz] in World frame
    - q: Orientation Quaternion (4) [w, x, y, z] mapping Body -> World
    - b_a: Accelerometer Bias (3) [b_ax, b_ay, b_az] in Body frame
    - b_g: Gyroscope Bias (3) [b_gx, b_gy, b_gz] in Body frame

Error State (15-dim):
    - delta_p: (3) Position error
    - delta_v: (3) Velocity error
    - delta_theta: (3) Orientation error angle-axis
    - delta_b_a: (3) Accel bias error
    - delta_b_g: (3) Gyro bias error
"""

import numpy as np


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """Generate 3x3 skew-symmetric matrix [v]x for cross product."""
    v = np.asarray(v).flatten()
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=np.float64)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion [w, x, y, z]."""
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions: q = q1 * q2, format [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)


def quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix R_wb (Body -> World)."""
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2)]
    ], dtype=np.float64)


def rot_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return quat_normalize(np.array([w, x, y, z], dtype=np.float64))


def axis_angle_to_quat(dtheta: np.ndarray) -> np.ndarray:
    """Convert small error axis-angle vector delta_theta to quaternion delta_q."""
    angle = np.linalg.norm(dtheta)
    if angle < 1e-10:
        return np.array([1.0, 0.5 * dtheta[0], 0.5 * dtheta[1], 0.5 * dtheta[2]], dtype=np.float64)
    axis = dtheta / angle
    w = np.cos(0.5 * angle)
    xyz = axis * np.sin(0.5 * angle)
    return np.array([w, xyz[0], xyz[1], xyz[2]], dtype=np.float64)


class RoverState:
    """Nominal kinematic state for all-weather ground vehicle."""

    def __init__(self,
                 p: np.ndarray = None,
                 v: np.ndarray = None,
                 q: np.ndarray = None,
                 b_a: np.ndarray = None,
                 b_g: np.ndarray = None):
        self.p = np.zeros(3, dtype=np.float64) if p is None else np.asarray(p, dtype=np.float64)
        self.v = np.zeros(3, dtype=np.float64) if v is None else np.asarray(v, dtype=np.float64)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64) if q is None else quat_normalize(q)
        self.b_a = np.zeros(3, dtype=np.float64) if b_a is None else np.asarray(b_a, dtype=np.float64)
        self.b_g = np.zeros(3, dtype=np.float64) if b_g is None else np.asarray(b_g, dtype=np.float64)

    @property
    def rot_matrix(self) -> np.ndarray:
        return quat_to_rot_matrix(self.q)

    @property
    def body_velocity(self) -> np.ndarray:
        """Translational velocity in Body frame: v_body = R_wb^T * v_world."""
        return self.rot_matrix.T @ self.v

    def copy(self) -> 'RoverState':
        return RoverState(
            p=self.p.copy(),
            v=self.v.copy(),
            q=self.q.copy(),
            b_a=self.b_a.copy(),
            b_g=self.b_g.copy()
        )

    def inject_error(self, delta_x: np.ndarray):
        """
        Inject 15-dim error state into nominal state (error-state reset):
        x_nominal = x_nominal (+) delta_x
        """
        dp = delta_x[0:3]
        dv = delta_x[3:6]
        dtheta = delta_x[6:9]
        dba = delta_x[9:12]
        dbg = delta_x[12:15]

        # Position and velocity addition
        self.p += dp
        self.v += dv

        # Quaternion multiplication for SO(3) retraction
        dq = axis_angle_to_quat(dtheta)
        self.q = quat_normalize(quat_multiply(self.q, dq))

        # Bias addition
        self.b_a += dba
        self.b_g += dbg

    def to_dict(self) -> dict:
        return {
            'pos': self.p.tolist(),
            'vel': self.v.tolist(),
            'quat': self.q.tolist(),
            'bias_acc': self.b_a.tolist(),
            'bias_gyr': self.b_g.tolist()
        }
