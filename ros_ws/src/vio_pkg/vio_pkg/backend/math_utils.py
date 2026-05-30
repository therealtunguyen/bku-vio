import numpy as np

def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """
    Returns the skew-symmetric matrix of a 3D vector.
    """
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0]
    ], dtype=np.float64)

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamiltonian quaternion multiplication.
    q = [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)

def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert a Hamilton quaternion [w, x, y, z] to a 3x3 rotation matrix.
    """
    q = normalize_quaternion(q)
    w, x, y, z = q
    
    return np.array([
        [1 - 2*y**2 - 2*z**2,     2*x*y - 2*w*z,         2*x*z + 2*w*y],
        [2*x*y + 2*w*z,         1 - 2*x**2 - 2*z**2,     2*y*z - 2*w*x],
        [2*x*z - 2*w*y,         2*y*z + 2*w*x,         1 - 2*x**2 - 2*y**2]
    ], dtype=np.float64)

def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a quaternion [w, x, y, z].
    """
    trace = np.trace(R)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
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
        
    return normalize_quaternion(np.array([w, x, y, z], dtype=np.float64))

def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion to have unit length.
    """
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm

def omega_mat(v: np.ndarray) -> np.ndarray:
    """
    Omega matrix used for quaternion kinematics
    """
    x, y, z = v
    return np.array([
        [0, -x, -y, -z],
        [x,  0,  z, -y],
        [y, -z,  0,  x],
        [z,  y, -x,  0]
    ], dtype=np.float64)
