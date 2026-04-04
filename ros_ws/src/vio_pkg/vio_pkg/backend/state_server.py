import numpy as np
from ..utils.common import State, ClonePose

class StateServer:
    def __init__(self):
        self.state = State(timestamp=0.0)
        # Covariance matrix P
        self.covariance = np.eye(15) # 15x15 for error state (pos, vel, ori, bg, ba)
        # Max number of clones in the sliding window
        self.max_window_size = 20
        
    def add_clone(self, timestamp: float, position: np.ndarray, quaternion: np.ndarray):
        """
        Add a camera state clone for the current state and expand the covariance matrix.
        """
        # TODO: Implement State Augmentation
        pass
        
    def remove_oldest_clone(self):
        """
        Remove the oldest camera state clone to keep the size stable.
        """
        # TODO: Implement marginalization of the oldest camera state
        pass
