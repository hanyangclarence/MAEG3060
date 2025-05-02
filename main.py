from classes.letters import *
from classes.base_letters import String, Letter, Stroke, get_2d_visualization
from test_ik import ik
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
    poses *= 0.5  # scale down the poses to fit the workspace of the robot
    
    
    
    # print(f"Time steps: {time_steps.shape}")
    # print(f"Poses: {poses.shape}")
    # print(f"Velocities: {velocities.shape}")
    # print(f"Accelerations: {accelerations.shape}")

    # get_2d_visualization(cuhk, "cuhk.png")
    
    all_thetas = []
    prev_thetas = np.zeros(6)  # initial joint angles
    orientation = np.eye(3)
    for pose in poses:
        thetas = ik(orientation, pose, curr_thetas=prev_thetas)
        all_thetas.append(thetas[None, ...])
        prev_thetas = thetas
    
    all_thetas = np.concatenate(all_thetas, axis=0)  # (N, 6)
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
    
    out_string = "joint_space;"
    for j in range(6):
        for v in all_thetas[:, j]:
            out_string += f"{float(v):.5f},"
        out_string = out_string[:-1] + ";"
    
    with open("joint_space.txt", "w") as f:
        f.write(out_string[:-1])