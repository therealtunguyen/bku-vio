import numpy as np
from typing import List
import scipy.linalg as linalg
from scipy.stats import chi2
from ..utils.common import FeatureTrack
from .state_server import StateServer
from .math_utils import quaternion_to_matrix, quaternion_multiply, normalize_quaternion, skew_symmetric

# Precompute chi-squared thresholds (95th percentile) indexed by DOF
_CHI2_THRESH = {dof: chi2.ppf(0.95, dof) for dof in range(2, 201)}

class MSCKFUpdater:
    def __init__(self, state_server: StateServer):
        self.state_server = state_server
        self.measurement_noise = 1e-4
        self.last_update_stats = {}
        self.last_batch_rejected = False

        # Runtime safety rails. These are intentionally conservative because a
        # single bad visual batch can destroy the inertial state.
        self.max_batch_dx_pos_norm = 0.5
        self.max_batch_dx_vel_norm = 1.0
        self.max_batch_dx_bias_norm = 0.25
        self.max_update_condition_number = 1e12

        # Euroc MAV dataset cam0 intrinsics (approx baseline)
        self.fx = 458.654
        self.fy = 457.296
        self.cx = 367.215
        self.cy = 248.375

    def _get_clone_sequence(self, feature: FeatureTrack):
        """
        Resolve a feature's observation timestamps to the current clone window.
        FeatureTrack.camera_states are frontend snapshots; the EKF update must
        use the authoritative clone poses stored in StateServer.
        """
        if len(feature.observations) != len(feature.camera_states):
            return None

        clone_by_timestamp = {
            clone.timestamp: (idx, clone)
            for idx, clone in enumerate(self.state_server.state.clone_poses)
        }

        clone_sequence = []
        for cam_pose in feature.camera_states:
            clone_entry = clone_by_timestamp.get(cam_pose.timestamp)
            if clone_entry is None:
                return None
            clone_sequence.append(clone_entry)
        return clone_sequence

    def process_mature_features(self, mature_features: List[FeatureTrack]):
        """
        Process the features that have been successfully tracked and are now ready.
        """
        stats = {
            "mature": len(mature_features),
            "too_short": 0,
            "triangulated": 0,
            "triangulation_failed": 0,
            "invalid_jacobian": 0,
            "gated_out": 0,
            "accepted": 0,
            "rows": 0,
            "dx_norm": 0.0,
            "dx_pos_norm": 0.0,
            "dx_vel_norm": 0.0,
            "dx_accel_bias_norm": 0.0,
            "batch_rejected": 0,
        }

        if not mature_features:
            self.last_update_stats = stats
            return

        H_stacked = []
        r_stacked = []

        for feature in mature_features:
            if len(feature.observations) < 3:
                stats["too_short"] += 1
                continue

            feature_3d = self.triangulate_feature(feature)
            if feature_3d is None:
                stats["triangulation_failed"] += 1
                continue
            stats["triangulated"] += 1

            H_x, H_f, r, valid = self.calc_residuals_and_jacobian(feature_3d, feature)
            if not valid:
                stats["invalid_jacobian"] += 1
                continue

            H_xo, r_o = self.null_space_projection(H_x, H_f, r)
            if H_xo is not None and r_o is not None:
                if self._gating_test(H_xo, r_o):
                    H_stacked.append(H_xo)
                    r_stacked.append(r_o)
                    stats["accepted"] += 1
                    stats["rows"] += H_xo.shape[0]
                else:
                    stats["gated_out"] += 1

        if not H_stacked:
            self.last_update_stats = stats
            return

        H_all = np.vstack(H_stacked)
        r_all = np.concatenate(r_stacked)

        dx = self.measurement_update(H_all, r_all)
        stats["dx_norm"] = float(np.linalg.norm(dx))
        stats["dx_pos_norm"] = float(np.linalg.norm(dx[0:3]))
        stats["dx_vel_norm"] = float(np.linalg.norm(dx[3:6]))
        stats["dx_accel_bias_norm"] = float(np.linalg.norm(dx[12:15]))
        stats["batch_rejected"] = int(self.last_batch_rejected)
        self.last_update_stats = stats

    def triangulate_feature(self, feature: FeatureTrack):
        """
        Estimate initial 3D position of the feature using DLT.
        """
        A = []
        clone_sequence = self._get_clone_sequence(feature)
        if clone_sequence is None:
            return None

        for obs, (_, cam_pose) in zip(feature.observations, clone_sequence):
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

            # Reject poorly triangulated points via mean reprojection error
            total_err = 0.0
            n_obs = len(feature.observations)
            for obs, (_, cam_pose) in zip(feature.observations, clone_sequence):
                R_cw = quaternion_to_matrix(cam_pose.quaternion).T
                p_c = R_cw @ (p_w - cam_pose.position)
                if p_c[2] < 1e-3:
                    return None
                px = self.fx * p_c[0] / p_c[2] + self.cx
                py = self.fy * p_c[1] / p_c[2] + self.cy
                total_err += np.sqrt((px - obs[0])**2 + (py - obs[1])**2)
            if total_err / n_obs > 5.0:
                return None

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

        clone_sequence = self._get_clone_sequence(feature)
        if clone_sequence is None:
            return None, None, None, False

        for i, (obs, (clone_idx, cam_pose)) in enumerate(zip(feature.observations, clone_sequence)):
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
            J_theta = skew_symmetric(f_c)
            J_p = -R_wc.T
            
            H_x_i = J_proj @ np.hstack([J_theta, J_p])
            H_x[2*i : 2*i+2, state_idx : state_idx+6] = H_x_i

        return H_x, H_f, r, True

    def _gating_test(self, H_xo: np.ndarray, r_o: np.ndarray) -> bool:
        """
        Chi-squared gating test (KumarRobotics msckf_vio gatingTest).
        Rejects features whose projected residual is statistically inconsistent
        with the current state covariance — catches bad triangulations and outliers.
        """
        dof = r_o.shape[0]
        if dof < 2:
            return False
        P = self.state_server.covariance
        S = H_xo @ P @ H_xo.T + np.eye(dof) * self.measurement_noise
        gamma = r_o @ np.linalg.solve(S, r_o)
        thresh = _CHI2_THRESH.get(dof, chi2.ppf(0.95, dof))
        return gamma < thresh

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
        self.last_batch_rejected = False

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
        if not np.all(np.isfinite(S)) or np.linalg.cond(S) > self.max_update_condition_number:
            self.last_batch_rejected = True
            return np.zeros(P.shape[0])

        K = P @ H_th.T @ linalg.inv(S)

        # Update State
        dx = K @ r_th
        if self._is_unreasonable_update(dx):
            self.last_batch_rejected = True
            return dx

        self.apply_state_update(dx)

        # Update Covariance using Joseph form for better numerical stability.
        I_KH = np.eye(P.shape[0]) - K @ H_th
        self.state_server.covariance = I_KH @ P @ I_KH.T + K @ R_n @ K.T
        # Enforce symmetry
        self.state_server.covariance = (self.state_server.covariance + self.state_server.covariance.T) / 2.0
        return dx

    def _is_unreasonable_update(self, dx: np.ndarray) -> bool:
        if not np.all(np.isfinite(dx)):
            return True
        if np.linalg.norm(dx[0:3]) > self.max_batch_dx_pos_norm:
            return True
        if np.linalg.norm(dx[3:6]) > self.max_batch_dx_vel_norm:
            return True
        if np.linalg.norm(dx[9:15]) > self.max_batch_dx_bias_norm:
            return True
        return False

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
