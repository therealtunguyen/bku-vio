import numpy as np
from typing import List
import scipy.linalg as linalg
from ..utils.common import FeatureTrack
from .state_server import StateServer
from .math_utils import quaternion_to_matrix, quaternion_multiply, normalize_quaternion, skew_symmetric

class MSCKFUpdater:
    def __init__(self, state_server: StateServer):
        self.state_server = state_server
        self.measurement_noise = 1e-4

        # Euroc MAV dataset cam0 intrinsics (approx baseline)
        self.fx = 458.654
        self.fy = 457.296
        self.cx = 367.215
        self.cy = 248.375

    def process_mature_features(self, mature_features: List[FeatureTrack]):
        """
        Process the features that have been successfully tracked and are now ready.
        """
        if not mature_features:
            return

        H_stacked = []
        r_stacked = []

        for feature in mature_features:
            if len(feature.camera_states) < 3:
                continue

            feature_3d = self.triangulate_feature(feature)
            if feature_3d is None:
                continue

            H_x, H_f, r, valid = self.calc_residuals_and_jacobian(feature_3d, feature)
            if not valid:
                continue

            H_xo, r_o = self.null_space_projection(H_x, H_f, r)
            if H_xo is not None and r_o is not None:
                H_stacked.append(H_xo)
                r_stacked.append(r_o)

        if not H_stacked:
            return

        H_all = np.vstack(H_stacked)
        r_all = np.concatenate(r_stacked)

        self.measurement_update(H_all, r_all)

    def triangulate_feature(self, feature: FeatureTrack):
        """
        Estimate initial 3D position of the feature using DLT.
        """
        A = []
        for obs, cam_pose in zip(feature.observations, feature.camera_states):
            R_wc = quaternion_to_matrix(cam_pose.quaternion)
            p_c = cam_pose.position
            # Projection matrix components
            R_cw = R_wc.T
            t = -R_cw @ p_c
            
            P_matrix = np.hstack([R_cw, t.reshape(3, 1)])
            P1 = P_matrix[0, :]
            P2 = P_matrix[1, :]
            P3 = P_matrix[2, :]
            
            # Convert pixel to normalized coordinate
            x = (obs[0] - self.cx) / self.fx
            y = (obs[1] - self.cy) / self.fy
            
            A.append(x * P3 - P1)
            A.append(y * P3 - P2)
            
        A = np.array(A)
        try:
            _, _, V = np.linalg.svd(A)
            # Prevent division by zero if homogeneous scale is very small
            if abs(V[-1, 3]) < 1e-6:
                return None
            p_w = V[-1, :3] / V[-1, 3] # Homogeneous to 3D
            return p_w
        except np.linalg.LinAlgError:
            return None

    def calc_residuals_and_jacobian(self, feature_3d: np.ndarray, feature: FeatureTrack):
        """
        Calculate reprojection error and Jacobian matrix (H).
        """
        n_obs = len(feature.observations)
        n_state = 15 + 6 * len(self.state_server.state.clone_poses)
        
        H_x = np.zeros((2 * n_obs, n_state))
        H_f = np.zeros((2 * n_obs, 3))
        r = np.zeros(2 * n_obs)

        # Mapping clone timestamps to its position in the state
        clone_timestamps = [c.timestamp for c in self.state_server.state.clone_poses]

        for i, (obs, cam_pose) in enumerate(zip(feature.observations, feature.camera_states)):
            if cam_pose.timestamp not in clone_timestamps:
                return None, None, None, False
                
            clone_idx = clone_timestamps.index(cam_pose.timestamp)
            state_idx = 15 + 6 * clone_idx

            R_wc = quaternion_to_matrix(cam_pose.quaternion)
            p_c = cam_pose.position
            
            # Feature purely in camera frame
            f_c = R_wc.T @ (feature_3d - p_c)
            # Cannot project if depth is negative or near zero
            if f_c[2] < 1e-3:
                return None, None, None, False

            x = f_c[0] / f_c[2]
            y = f_c[1] / f_c[2]
            z_hat = np.array([x, y])

            # Reprojection Error (normalized coordinate)
            # The observation from frontend is taking in raw pixels, so we normalize it here!
            obs_norm = np.array([
                (obs[0] - self.cx) / self.fx,
                (obs[1] - self.cy) / self.fy
            ])
            r[2*i : 2*i+2] = obs_norm - z_hat

            # Jacobian of projection w.r.t 3D feature in Camera Frame
            J_proj = (1.0 / f_c[2]) * np.array([
                [1.0, 0.0, -x],
                [0.0, 1.0, -y]
            ])

            # Jacobian w.r.t feature position in World Frame
            H_f_i = J_proj @ R_wc.T
            H_f[2*i : 2*i+2, :] = H_f_i

            # Jacobian w.r.t camera state
            # state var = [delta_theta_c, delta_p_c]
            J_theta = -skew_symmetric(f_c)
            J_p = -R_wc.T
            
            H_x_i = J_proj @ np.hstack([J_theta, J_p])
            H_x[2*i : 2*i+2, state_idx : state_idx+6] = H_x_i

        return H_x, H_f, r, True

    def null_space_projection(self, H_x, H_f, r):
        """
        Project onto the left nullspace of the feature Jacobian to remove
        the 3D feature representation (Marginalization).
        """
        # QR Decomposition
        Q, R = linalg.qr(H_f, mode='full')
        
        # Left nullspace is columns of Q beyond rank (which is 3)
        Q_n = Q[:, 3:]
        
        H_xo = Q_n.T @ H_x
        r_o = Q_n.T @ r
        
        return H_xo, r_o

    def measurement_update(self, H_all, r_all):
        """
        Standard Kalman Filter update to correct State and Covariance.
        """
        # Compress H and r using Thin QR to speed up matrix inversion
        if H_all.shape[0] > H_all.shape[1]:
            Q, R = linalg.qr(H_all, mode='economic')
            H_th = R
            r_th = Q.T @ r_all
        else:
            H_th = H_all
            r_th = r_all

        P = self.state_server.covariance
        
        # Kalman Gain
        R_n = np.eye(H_th.shape[0]) * self.measurement_noise
        S = H_th @ P @ H_th.T + R_n
        K = P @ H_th.T @ linalg.inv(S)

        # Update State
        dx = K @ r_th
        self.apply_state_update(dx)

        # Update Covariance
        self.state_server.covariance = (np.eye(P.shape[0]) - K @ H_th) @ P
        # Enforce symmetry
        self.state_server.covariance = (self.state_server.covariance + self.state_server.covariance.T) / 2.0

    def apply_state_update(self, dx: np.ndarray):
        """
        Apply Error State update dx to nominal state
        """
        st = self.state_server.state
        
        # IMU state update
        # dx_imu = [dp(0:3), dv(3:6), dtheta(6:9), dbg(9:12), dba(12:15)]
        st.position += dx[0:3]
        st.velocity += dx[3:6]
        
        dtheta = dx[6:9]
        dq = np.concatenate(([1.0], 0.5 * dtheta)) # Approx exp(theta)
        dq = normalize_quaternion(dq)
        st.quaternion = normalize_quaternion(quaternion_multiply(st.quaternion, dq))
        
        st.gyro_bias += dx[9:12]
        st.accel_bias += dx[12:15]

        # Clone state updates
        for i, clone in enumerate(st.clone_poses):
            idx = 15 + i*6
            dtheta_c = dx[idx : idx+3]
            dp_c = dx[idx+3 : idx+6]
            
            dq_c = np.concatenate(([1.0], 0.5 * dtheta_c))
            dq_c = normalize_quaternion(dq_c)
            clone.quaternion = normalize_quaternion(quaternion_multiply(clone.quaternion, dq_c))
            clone.position += dp_c
