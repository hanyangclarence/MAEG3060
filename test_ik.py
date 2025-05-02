import numpy as np
from test_fk import get_T, FK_single


def base_robot_ik_sec1(xp, yp, zp, l1, l2, l3, curr_theta3=None):
    theta1 = np.arctan2(yp, xp)
    
    A = xp * np.cos(theta1) + yp * np.sin(theta1)
    B = zp - l1
    
    theta3 = np.arccos(
        np.clip((A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3), -1, 1)
    )
    if curr_theta3 is not None:
        diff = np.abs(np.sin(theta3) - np.sin(curr_theta3)) + np.abs(np.cos(theta3) - np.cos(curr_theta3))
        diff_neg = np.abs(np.sin(-theta3) - np.sin(curr_theta3)) + np.abs(np.cos(-theta3) - np.cos(curr_theta3))
        if diff_neg < diff:
            theta3 = -theta3
    theta2 = np.arctan2(
        B * (l2 + l3 * np.cos(theta3)) - A * l3 * np.sin(theta3),
        A * (l2 + l3 * np.cos(theta3)) + B * l3 * np.sin(theta3)
    )
    
    return theta1, theta2, theta3


def ik_sec1(xp, yp, zp, curr_theta3=None):
    phi_1 = np.arctan(30 / 264)
    phi_2 = np.arctan(30 / 258)
    
    l1 = 159
    l2 = np.sqrt(30**2 + 264**2)
    l3 = np.sqrt(30**2 + 258**2)
    
    curr_alpha3 = phi_1 + phi_2 - np.pi / 4 - curr_theta3 if curr_theta3 is not None else None
    
    theta1, alpha_2, alpha_3 = base_robot_ik_sec1(xp, yp, zp, l1, l2, l3, curr_theta3=curr_alpha3)
    
    theta2 = np.pi / 2 - phi_1 - alpha_2
    theta3 = phi_1 + phi_2 - alpha_3 - np.pi / 4
    
    return theta1, theta2, theta3


def ik(R_6_0, t, curr_thetas=None):
    assert R_6_0.shape == (3, 3), "Expected 3x3 rotation matrix"
    assert t.shape == (3,), "Expected 3x1 translation vector"
    if curr_thetas is not None:
        assert len(curr_thetas) == 6, "Expected 6 joint angles"
    
    # get the ee pose of the first section
    t_sec1 = t - R_6_0 @ np.array([0, 0, 123])
    xp = t_sec1[0]
    yp = t_sec1[1]
    zp = t_sec1[2]
    
    theta1, theta2, theta3 = ik_sec1(xp, yp, zp, curr_theta3=curr_thetas[2] if curr_thetas is not None else None)
    
    arm_ang = np.arctan2(30, 264)
    dh_table = np.array([
        [0, 0, 0, theta1],
        [-np.pi / 2, 0, 0, theta2 - np.pi / 2 + arm_ang],
        [0, 265.69, 0, theta3 - np.pi / 4 - arm_ang],
    ])
    R_0_3 = get_T(0, 3, dh_table)[:3, :3]
    
    R_6_3 = R_0_3 @ R_6_0
    
    theta4 = - np.arctan2(R_6_3[2, 2], R_6_3[0, 2])
    theta5 = np.arccos(np.clip(R_6_3[1, 2], -1, 1)) 
    theta6 = - np.arctan2(R_6_3[1, 1], R_6_3[1, 0])
    
    if curr_thetas is not None:
        diff = np.abs(np.sin(theta5) - np.sin(curr_thetas[4])) + np.abs(np.cos(theta5) - np.cos(curr_thetas[4]))
        diff_neg = np.abs(np.sin(-theta5) - np.sin(curr_thetas[4])) + np.abs(np.cos(-theta5) - np.cos(curr_thetas[4]))
        if diff_neg < diff:
            theta5 = -theta5
        for theta4_normed in [theta4, theta4 - np.pi, theta4 + np.pi]:
            if np.abs(theta4_normed - curr_thetas[3]) < np.abs(theta4 - curr_thetas[3]):
                theta4 = theta4_normed
        for theta6_normed in [theta6, theta6 - np.pi, theta6 + np.pi]:
            if np.abs(theta6_normed - curr_thetas[5]) < np.abs(theta6 - curr_thetas[5]):
                theta6 = theta6_normed
    
    thetas = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    for i, theta in enumerate(thetas):
        if np.abs(theta) > np.pi - 0.01:
            theta = theta - 2 * np.pi if theta > 0 else theta + 2 * np.pi
            thetas[i] = theta
    
    return thetas


if __name__ == "__main__":
    thetas = np.array([-0.5, 1.2, 1.3, np.pi / 3, np.pi / 7, -np.pi / 4])
    pose_gripper, orientation = FK_single(thetas)
    
    # print("Pose of gripper:", pose_gripper)
    # print("Orientation of gripper:", orientation)
    
    thetas_ik = ik(orientation, pose_gripper, curr_thetas=thetas)
    print("FK joint angles:", thetas)
    print("IK joint angles:", thetas_ik)
