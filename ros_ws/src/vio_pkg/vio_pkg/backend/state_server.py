import numpy as np
import threading
from ..utils.common import State, ClonePose
from .math_utils import quaternion_to_matrix, matrix_to_quaternion, skew_symmetric

VALID_CAMERA_EXTRINSICS_CONVENTIONS = (
    "camera_in_imu",
    "imu_in_camera",
)


def canonicalize_camera_extrinsics(
    R,
    t,
    *,
    convention: str = "camera_in_imu",
):
    """
    Normalize input extrinsics to the internal camera-in-IMU convention.

    Internal representation:
      p_I = R_IC @ p_C + t_IC
    which means:
      R_IC rotates camera-frame vectors into the IMU frame
      t_IC is the camera origin expressed in the IMU frame
    """
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)

    if R.shape != (3, 3):
        raise ValueError("R must be a 3x3 rotation matrix")
    if t.shape != (3,):
        raise ValueError("t must be a 3-vector")
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        raise ValueError("R must be orthonormal")
    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
        raise ValueError("R must have determinant +1")

    if convention == "camera_in_imu":
        return R, t
    if convention == "imu_in_camera":
        R_ic = R.T
        t_ic = -(R_ic @ t)
        return R_ic, t_ic
    raise ValueError(
        "convention must be one of "
        f"{VALID_CAMERA_EXTRINSICS_CONVENTIONS}"
    )


class StateServer:
    def __init__(self, R_IC=None, t_IC=None, camera_extrinsics_convention="camera_in_imu"):
        self.state = State(timestamp=0.0)
        
        # Euroc V1_01_easy cam0 to imu0 extrinsics (Default fallback)
        default_R_IC = np.array([
            [ 0.0148655429818, -0.999880929698,  0.004140296794],
            [ 0.999557249008,  0.014967213324,  0.025715529948],
            [-0.0257744366974, 0.003756188357,  0.999660727108]
        ])
        default_t_IC = np.array([-0.0216401455, -0.0646769868, 0.0098107306])
        self.set_camera_extrinsics(
            default_R_IC if R_IC is None else R_IC,
            default_t_IC if t_IC is None else t_IC,
            convention=camera_extrinsics_convention,
        )
        
        # Covariance matrix P
        # 15x15 for error state
        # pos (0:3), vel (3:6), ori (6:9), bg (9:12), ba (12:15)
        self.covariance = np.zeros((15, 15))
        
        # Initial covariance uncertainties
        self.covariance[0:3, 0:3] = np.eye(3) * 1e-4
        self.covariance[3:6, 3:6] = np.eye(3) * 1e-4
        self.covariance[6:9, 6:9] = np.eye(3) * 1e-4
        self.covariance[9:12, 9:12] = np.eye(3) * 1e-3
        self.covariance[12:15, 12:15] = np.eye(3) * 1e-2

        # Max number of clones in the sliding window
        self.max_window_size = 20
        
        # Concurrency Lock
        self.lock = threading.RLock()

    def set_camera_extrinsics(
        self,
        R_IC,
        t_IC,
        *,
        convention: str = "camera_in_imu",
    ) -> None:
        canonical_R_IC, canonical_t_IC = canonicalize_camera_extrinsics(
            R_IC,
            t_IC,
            convention=convention,
        )
        self.R_IC = canonical_R_IC
        self.t_IC = canonical_t_IC
        self.camera_extrinsics_convention = "camera_in_imu"
        self.input_camera_extrinsics_convention = convention
        
    def add_clone(self, timestamp: float, position: np.ndarray, quaternion: np.ndarray):
        """
        Add a camera state clone for the current state and expand the covariance matrix.
        Transforms the IMU nominal state into a Camera nominal state using Euroc Extrinsics.
        """
        # Convert IMU state to Camera State using extrinsics
        R_WI = quaternion_to_matrix(quaternion)
        R_WC = R_WI @ self.R_IC
        
        cam_position = position + R_WI @ self.t_IC
        cam_quaternion = matrix_to_quaternion(R_WC)
        
        # Create a new clone pose
        new_clone = ClonePose(
            timestamp=timestamp,
            position=cam_position,
            quaternion=cam_quaternion
        )
        self.state.clone_poses.append(new_clone)

        # Expand covariance matrix
        # New clone state error is [delta_theta_c; delta_p_c]
        # delta_theta_C = R_IC^T * delta_theta_I
        P = self.covariance
        n_rows = P.shape[0]
        
        J_new = np.zeros((6, n_rows))
        J_new[0:3, 6:9] = self.R_IC.T        # delta_theta_c = R_IC^T * delta_theta_I
        J_new[3:6, 0:3] = np.eye(3)
        J_new[3:6, 6:9] = -R_WI @ skew_symmetric(self.t_IC)
        
        P_CC_new = J_new @ P @ J_new.T
        P_IC_new = P @ J_new.T
        
        # Construct augmented P
        P_augmented = np.zeros((n_rows + 6, n_rows + 6))
        P_augmented[:n_rows, :n_rows] = P
        P_augmented[:n_rows, n_rows:] = P_IC_new
        P_augmented[n_rows:, :n_rows] = P_IC_new.T
        P_augmented[n_rows:, n_rows:] = P_CC_new
        
        self.covariance = (P_augmented + P_augmented.T) / 2.0
        
        # Marginalize oldest clone if window is too large
        while len(self.state.clone_poses) > self.max_window_size:
            self.remove_oldest_clone()

    def remove_oldest_clone(self):
        """
        Remove the oldest camera state clone to keep the size stable.
        """
        if not self.state.clone_poses:
            return
            
        # The oldest clone is the first one in the list
        self.state.clone_poses.pop(0)
        
        # It's located right after the IMU state in the covariance matrix (indices 15 to 20)
        P = self.covariance
        n_rows = P.shape[0]
        
        indices_to_keep = list(range(15)) + list(range(21, n_rows))
        
        # Slice the covariance matrix to drop rows and columns 15:20
        self.covariance = P[np.ix_(indices_to_keep, indices_to_keep)]

    def clear_clones(self):
        """Drop all camera clones and shrink covariance back to the IMU state."""
        self.state.clone_poses.clear()
        self.covariance = self.covariance[:15, :15].copy()
