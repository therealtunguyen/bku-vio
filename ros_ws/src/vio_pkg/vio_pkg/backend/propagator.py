import numpy as np
from typing import List
from ..utils.common import ImuData
from .state_server import StateServer

class ImuPropagator:
    def __init__(self, state_server: StateServer):
        self.state_server = state_server
        self.gravity = np.array([0, 0, -9.81])

    def propagate(self, imu_data_list: List[ImuData]):
        """
        Perform RK4 integration to predict the new State.
        Calculate Error-State Jacobian (Phi) and propagate Covariance.
        """
        for imu in imu_data_list:
            # TODO: Implement IMU kinematics (Predict phase)
            # TODO: Update state_server.state (Position, Velocity, Quaternion)
            # TODO: Propagate state_server.covariance P = Phi * P * Phi^T + Q
            pass
