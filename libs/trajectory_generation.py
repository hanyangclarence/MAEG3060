import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import typing as tp

def generate_trajectory_3d(
    pos_list: tp.List[np.ndarray],
	time_list: tp.List[float],
	frequency: int = 50,
) -> tp.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
	"""
	Generate a 3D trajectory based on given positions and time points.
	Default start and end velocities are 0, and the trajectory is cubic.

	Args:
		pos_list (List[np.ndarray]): List of 3D positions (x, y, z).
		time_list (List[float]): List of time points for each position.
		frequency (int): Frequency of the trajectory in Hz.

	Returns:
		time_steps: (np.ndarray): Array of time steps, list of floats, len T
		poses: np.ndarray (T, 3)
		velocities: np.ndarray (T, 3)
		accelerations: np.ndarray (T, 3)
	"""
	pass