import numpy as np
import dm_control

def clip_to_valid_state(physics: dm_control.mjcf.physics.Physics, qpos: np.array):
    """
    This function returns qpos with every value clipped to the allowable joint range as specified
    in the MJCF. 
    """
    qpos_clipped = qpos.copy()

    for joint_idx in range(physics.model.njnt):
        joint_range = physics.model.jnt_range[joint_idx]

        qpos_clipped[physics.model.jnt_qposadr[joint_idx]] = np.clip(
            qpos_clipped[physics.model.jnt_qposadr[joint_idx]], 
            joint_range[0],
            joint_range[1])

    return qpos_clipped

def quaternion_error_naive(current_quat: np.array, target_quat: np.array):
    """
    Rough orientation error between two quaternions.
    This is just a rough measure and doesn't work well for
    large angles, so you might want to consider using
    something more advanced to compare quaternions.
    """
    q_diff = quat_multiply(target_quat, quat_conjugate(current_quat))
    # Ignore w
    return q_diff[1:4]

def quat_multiply(q1: np.array, q2: np.array):
    """Multiply two quaternions, returning q1*q2 in [w, x, y, z] format."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2
    return np.array([w, x, y, z])

def quat_conjugate(q: np.array):
    """Return quaternion conjugate: [w, -x, -y, -z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])