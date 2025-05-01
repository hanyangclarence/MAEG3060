import numpy as np


def get_T_one_frame(from_dim: int, to_dim: int, dh_table: np.ndarray) -> np.ndarray:
    # Get the transformation matrix from one frame to another
    assert from_dim <= 6 and to_dim >= 0, f"Invalid dimensions: {from_dim}, {to_dim}"
    assert from_dim == to_dim + 1, f"from_dim: {from_dim}, to_dim: {to_dim}"
    
    dh_params = dh_table[from_dim - 1]
    alpha, a, d, theta = dh_params
    return np.array([
        [np.cos(theta),                 -np.sin(theta),                0,             a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

def get_T(from_dim: int, to_dim: int, dh_table: np.ndarray) -> np.ndarray:
    assert 0 <= from_dim <= 6 and 0 <= to_dim <= 6, f"Invalid dimensions: {from_dim}, {to_dim}"
    assert from_dim != to_dim, f"from_dim: {from_dim}, to_dim: {to_dim}"
    
    if from_dim > to_dim:
        T = np.eye(4)
        for i in range(to_dim, from_dim):
            T = T @ get_T_one_frame(i + 1, i, dh_table)
    else:
        T = np.eye(4)
        for i in range(from_dim, to_dim):
            T = T @ get_T_one_frame(i + 1, i, dh_table)
        T = np.linalg.inv(T)
    return T


def FK_single(thetas: np.ndarray):
    assert len(thetas) == 6, "Expected 6 joint angles"
    arm_ang = np.arctan2(30, 264)
    dh_table = np.array([
        [0, 0, 159, thetas[0]],
        [-np.pi / 2, 0, 0, thetas[1] - np.pi / 2 + arm_ang],
        [0, 265.69, 0, thetas[2] - np.pi / 4 - arm_ang],
        [-np.pi / 2, 30, 258, thetas[3]],
        [np.pi / 2, 0, 0, thetas[4]],
        [-np.pi / 2, 0, 0, thetas[5]],
    ])
    
    R_6_0 = get_T(6, 0, dh_table)
    pose_gripper = (R_6_0 @ np.array([0, 0, 123, 1]))[:3]
    orientation = R_6_0[:3, :3]
    return pose_gripper, orientation
    
    

if __name__ == "__main__":
    # Test the FK function with some joint angles
    thetas = np.array([0, 0, 0, 0, 0, 0])
    pose_gripper, orientation = FK_single(thetas)
    print("Pose of gripper:", pose_gripper)
    print("Orientation of gripper:", orientation)
