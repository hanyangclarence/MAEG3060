from classes.letters import *
from classes.base_letters import String, Letter, Stroke, get_2d_visualization
from test_ik import ik
from test_fk import FK_single
import numpy as np

if __name__ == "__main__":
    cuhk = String(
        letters=[
            get_C(),
            get_U(),
            get_H(),
            get_K()
        ]
    )

    time_steps, poses, velocities, accelerations = cuhk.get_trajectory()
    
    # rotate the poses by -90 degrees around the z-axis
    rotation_matrix = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    poses = np.dot(poses, rotation_matrix.T)  # (N, 3)
    poses = poses + np.array([100, 200, 0])
    
    all_thetas = []
    prev_thetas = np.zeros(6)  # initial joint angles
    orientation = np.array(
        [[0, 0, -1],
        [1, 0, 0],
        [0, -1, 0]]
    )
    for pose in poses:
        thetas = ik(orientation, pose, curr_thetas=prev_thetas)
        all_thetas.append(thetas[None, ...])
        prev_thetas = thetas
    
    all_thetas = np.concatenate(all_thetas, axis=0)  # (N, 6)
    
    # use fk to verify
    recon_poses = []
    for i in range(all_thetas.shape[0]):
        pose_gripper, orientation_recon = FK_single(all_thetas[i])
        recon_poses.append(pose_gripper[None, ...])
    recon_poses = np.concatenate(recon_poses, axis=0)  # (N, 3)
        
    
    
    
    all_thetas = np.rad2deg(all_thetas)  # convert to degrees
    
    
    
    # plot the joint angles
    import matplotlib.pyplot as plt
    plt.figure(figsize=(40, 10))
    for j in range(6):
        plt.plot(time_steps, all_thetas[:, j], label=f"Joint {j + 1}")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angles (degrees)")
    plt.title("Joint Angles Over Time")
    plt.legend()
    plt.savefig("joint_angles.png")
    
    out_string = "angle;"
    for j in range(6):
        for v in all_thetas[:, j]:
            out_string += f"{float(v)},"
        out_string = out_string[:-1] + ";"
    
    with open("joint_space.txt", "w") as f:
        f.write(out_string)