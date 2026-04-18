import numpy as np
from typing import List
from ..utils.common import ImuData, State
from .state_server import StateServer
from .math_utils import quaternion_to_matrix, quaternion_multiply, normalize_quaternion, omega_mat, skew_symmetric

class ImuPropagator:
    def __init__(self, state_server: StateServer):
        self.state_server = state_server
        self.gravity = np.array([0, 0, -9.81])
        
        # Continuous time noise density
        self.noise_gyro = 1e-3
        self.noise_acc = 1e-2
        self.noise_gyro_bias = 5e-5
        self.noise_acc_bias = 1e-4
        
        Q_c = np.zeros((12, 12))
        Q_c[0:3, 0:3] = (self.noise_gyro ** 2) * np.eye(3)
        Q_c[3:6, 3:6] = (self.noise_acc ** 2) * np.eye(3)
        Q_c[6:9, 6:9] = (self.noise_gyro_bias ** 2) * np.eye(3)
        Q_c[9:12, 9:12] = (self.noise_acc_bias ** 2) * np.eye(3)
        self.Q_c = Q_c

    def predict_mean_state(self, state: State, w: np.ndarray, a: np.ndarray, dt: float):
        """
        Predict nominal state using 4th order Runge Kutta
        """
        q = state.quaternion
        p = state.position
        v = state.velocity
        bg = state.gyro_bias
        ba = state.accel_bias

        def k_dot(q_, v_):
            R_ = quaternion_to_matrix(q_)
            q_dot = 0.5 * omega_mat(w - bg) @ q_
            v_dot = R_ @ (a - ba) + self.gravity
            p_dot = v_
            return q_dot, p_dot, v_dot

        k1_q, k1_p, k1_v = k_dot(q, v)
        k2_q, k2_p, k2_v = k_dot(q + 0.5 * dt * k1_q, v + 0.5 * dt * k1_v)
        k3_q, k3_p, k3_v = k_dot(q + 0.5 * dt * k2_q, v + 0.5 * dt * k2_v)
        k4_q, k4_p, k4_v = k_dot(q + dt * k3_q, v + dt * k3_v)

        q_next = q + (dt / 6.0) * (k1_q + 2*k2_q + 2*k3_q + k4_q)
        v_next = v + (dt / 6.0) * (k1_v + 2*k2_v + 2*k3_v + k4_v)
        p_next = p + (dt / 6.0) * (k1_p + 2*k2_p + 2*k3_p + k4_p)

        return normalize_quaternion(q_next), p_next, v_next

    def propagate(self, imu_data_list: List[ImuData]):
        """
        Perform RK4 integration to predict the new State.
        Calculate Error-State Jacobian (Phi) and propagate Covariance.
        """
        if not imu_data_list:
            return

        for i in range(len(imu_data_list)):
            imu = imu_data_list[i]
            
            # Timestamp check
            if self.state_server.state.timestamp <= 0.0:
                self.state_server.state.timestamp = imu.timestamp
                continue
                
            dt = imu.timestamp - self.state_server.state.timestamp
            if dt <= 0:
                continue

            state = self.state_server.state
            w = imu.gyro
            a = imu.accel
            
            # Predict Mean State
            q_next, p_next, v_next = self.predict_mean_state(state, w, a, dt)
            
            # Compute Discrete Error State Jacobian (Phi)
            R = quaternion_to_matrix(state.quaternion)
            a_hat = a - state.accel_bias
            w_hat = w - state.gyro_bias
            
            F = np.zeros((15, 15))
            # pos (0:3), vel (3:6), ori (6:9), gyro_bias (9:12), accel_bias (12:15)
            F[0:3, 3:6] = np.eye(3)
            # dv_dot wrt ori
            F[3:6, 6:9] = R @ skew_symmetric(a_hat)
            # dv_dot wrt accel_bias
            F[3:6, 12:15] = -R
            # dori_dot wrt ori
            F[6:9, 6:9] = -skew_symmetric(w_hat)
            # dori_dot wrt gyro_bias
            F[6:9, 9:12] = -np.eye(3)

            # Approximation for exp(F*dt)
            Phi = np.eye(15) + F * dt + 0.5 * (F @ F) * (dt ** 2)

            G = np.zeros((15, 12))
            G[3:6, 3:6] = -R
            G[6:9, 0:3] = -np.eye(3)
            G[9:12, 6:9] = np.eye(3)
            G[12:15, 9:12] = np.eye(3)
            
            with self.state_server.lock:
                # Propagate Covariance
                # Q_d = G * Q_c * G^T * dt
                Q_d = (Phi @ G) @ self.Q_c @ (Phi @ G).T * dt
                
                # P_next = Phi * P * Phi^T + Q_d
                P = self.state_server.covariance
                # IMU state is the first 15x15 block
                P_imu = P[0:15, 0:15]
                P_imu_next = Phi @ P_imu @ Phi.T + Q_d
                
                # Update cross correlations (IMU-Camera)
                if P.shape[0] > 15:
                    P_imu_cam = P[0:15, 15:]
                    P_imu_cam_next = Phi @ P_imu_cam
                    P[0:15, 15:] = P_imu_cam_next
                    P[15:, 0:15] = P_imu_cam_next.T

                P[0:15, 0:15] = P_imu_next
                
                # Enforce symmetry
                self.state_server.covariance = (P + P.T) / 2.0
                
                # Update state values
                self.state_server.state.quaternion = q_next
                self.state_server.state.position = p_next
                self.state_server.state.velocity = v_next
                self.state_server.state.timestamp = imu.timestamp
