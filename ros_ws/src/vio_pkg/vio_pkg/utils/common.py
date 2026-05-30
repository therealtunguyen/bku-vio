import numpy as np
from dataclasses import dataclass, field
from typing import List

@dataclass
class ImuData:
    timestamp: float
    accel: np.ndarray  # [x, y, z]
    gyro: np.ndarray   # [x, y, z]

@dataclass
class CameraPose:
    timestamp: float
    position: np.ndarray
    quaternion: np.ndarray # [w, x, y, z]

@dataclass
class FeatureTrack:
    feature_id: int
    observations: List[np.ndarray] = field(default_factory=list) # List of image points [u, v]
    camera_states: List[CameraPose] = field(default_factory=list) # Camera states where this feature was observed

@dataclass
class ClonePose:
    timestamp: float
    position: np.ndarray
    quaternion: np.ndarray

@dataclass
class State:
    timestamp: float
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0])) # [w, x, y, z]
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    clone_poses: List[ClonePose] = field(default_factory=list) # Camera states in sliding window
