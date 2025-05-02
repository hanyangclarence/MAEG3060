from classes.letters import *
from classes.base_letters import String, Letter, Stroke, get_2d_visualization
from test_ik import ik
from test_fk import FK_single
import numpy as np
import time
import socket


def udp_client(server_ip, server_port, message):
    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send the message to the server
        client_socket.sendto(message.encode(), (server_ip, server_port))
        print(f"Command sent to {server_ip}:{server_port}, command: {message}")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the socket
        client_socket.close()


def send_command(server_ip, server_port, angle_list):
    '''
    this function is used to send the command to the robot. If you are using the simulator, you need to set the robot to Server Mode in the setting page.
    angle_list: list of angles
    '''
    message = ",".join([str(i) for i in angle_list])
    udp_client(server_ip, server_port, message)


if __name__ == "__main__":
    frequency = 50

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
    poses = poses + np.array([100, 300, 60])
    
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

    # add a linear trajectory from 0 pose to the first pose
    start_values = []
    for i in range(6):
        start_i = 0
        end_i = all_thetas[0, i]
        # use linear interpolation to generate the trajectory of 100 steps
        start_values.append(np.linspace(start_i, end_i, 100)[None, ...])
    start_values = np.concatenate(start_values, axis=0)  # (6, 100)
    start_values = start_values.T  # (100, 6)
    all_thetas = np.concatenate([start_values, all_thetas], axis=0)  # (N+100, 6)
    
    # use fk to verify
    recon_poses = []
    for i in range(all_thetas.shape[0]):
        pose_gripper, orientation_recon = FK_single(all_thetas[i])
        recon_poses.append(pose_gripper[None, ...])
    recon_poses = np.concatenate(recon_poses, axis=0)  # (N, 3)
        
    all_thetas = np.rad2deg(all_thetas)  # convert to degrees
    
    
    
    # plot the joint angles
    # import matplotlib.pyplot as plt
    # plt.figure(figsize=(40, 10))
    # for j in range(6):
    #     plt.plot(time_steps, all_thetas[:, j], label=f"Joint {j + 1}")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Joint Angles (degrees)")
    # plt.title("Joint Angles Over Time")
    # plt.legend()
    # plt.savefig("joint_angles.png")
    
    out_string = "angle;"
   
    for j in range(6):
        for v in all_thetas[:, j]:
            out_string += f"{float(v)},"
        out_string = out_string[:-1] + ";"
    
    with open("joint_space.txt", "w") as f:
        f.write(out_string)
    
    server_ip = "localhost" 
    server_port = 5000  # Replace with the server's port
    for i in range(all_thetas.shape[0]):
        angle_list = all_thetas[i, :].tolist()
        send_command(server_ip, server_port, angle_list)
        time.sleep(0.02)