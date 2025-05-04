import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import typing as tp

def generate_trajectory_3d(pos_list, time_list,	frequency, t_b):
    
    pos_array = np.array(pos_list)  # Shape: (N, 3), N = number of waypoints
    time_array = np.array(time_list)  # Shape: (N,)

    if len(pos_array) != len(time_array):
        raise ValueError("pos_list and time_list must have the same length")
    if len(pos_array) < 2:
        raise ValueError("At least two positions are required")
    if not all(time_array[i] < time_array[i + 1] for i in range(len(time_array) - 1)):
        raise ValueError("time_list must be strictly increasing")

    # total duration and time steps
    total_duration = time_array[-1] - time_array[0]
    num_steps = int(total_duration * frequency) + 1
    time_steps = np.linspace(time_array[0], time_array[-1], num_steps)

    # initialize output arrays
    poses = np.zeros((num_steps, 3))
    velocities = np.zeros((num_steps, 3))
    accelerations = np.zeros((num_steps, 3))

    # Number of segments
    N = len(pos_array) - 1

    # dim as each dimension (x, y, z)
    for dim in range(3):
        positions = pos_array[:, dim]  # shape: (N+1,)
        segment_acc = np.zeros(N)  # accelerations for each segment
        segment_vel = np.zeros(N)  # linear velocities for each segment

        # get acceleration and linear velocity for each segment
        for i in range(N):
            t_final = time_array[i + 1] - time_array[i]
            pos_offset = positions[i]
            pos_final = positions[i + 1]
            # step acceleration formula
            segment_acc[i] = (pos_offset - pos_final) / (t_b**2 - t_final * t_b)
            # linear velocity at the end of first blend
            segment_vel[i] = segment_acc[i] * t_b

        # trajectory for each time step
        for t_idx, t in enumerate(time_steps):
            for i in range(N):
                t_start = time_array[i]
                t_final = time_array[i + 1]
                t_rel = t - t_start
                if t_start <= t <= t_final:
                    pos_offset = positions[i]
                    pos_final = positions[i + 1]
                    acc = segment_acc[i]
                    v_linear = segment_vel[i]

                    # First blend
                    if 0 <= t_rel < t_b:
                        poses[t_idx, dim] = (acc / 2) * t_rel**2 + pos_offset
                        velocities[t_idx, dim] = acc * t_rel
                        accelerations[t_idx, dim] = acc
                    # Linear vel
                    elif t_b <= t_rel < t_final - t_start - t_b:
                        pos_blend = (acc / 2) * t_b**2 + pos_offset
                        poses[t_idx, dim] = acc * t_b * (t_rel - t_b) + pos_blend
                        velocities[t_idx, dim] = v_linear
                        accelerations[t_idx, dim] = 0
                    # Second blend
                    elif t_final - t_start - t_b <= t_rel <= t_final- t_start:
                        t_from_end = t_rel + t_start - t_final
                        poses[t_idx, dim] = -(acc / 2) * t_from_end**2 + pos_final
                        velocities[t_idx, dim] = -acc * t_from_end
                        accelerations[t_idx, dim] = -acc
                    break                
                
    return time_steps, poses, velocities, accelerations 

# def generate_trajectory_3d(
#     pos_list: tp.List[np.ndarray],
# 	time_list: tp.List[float],
# 	frequency: int = 50,
# ) -> tp.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
# 	"""
# 	Generate a 3D trajectory based on given positions and time points.
# 	Default start and end velocities are 0, and the trajectory is cubic.

# 	Args:
# 		pos_list (List[np.ndarray]): List of 3D positions (x, y, z).
# 		time_list (List[float]): List of time points for each position.
# 		frequency (int): Frequency of the trajectory in Hz.

# 	Returns:
# 		time_steps: (np.ndarray): Array of time steps, list of floats, len T
# 		poses: np.ndarray (T, 3)
# 		velocities: np.ndarray (T, 3)
# 		accelerations: np.ndarray (T, 3)
# 	"""
# 	pass
