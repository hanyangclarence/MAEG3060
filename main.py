from classes.letters import *
from classes.base_letters import String, get_2d_visualization
from libs.inverse_kinematics import ik
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
    cuhk = String(
        letters=[
            get_C(),
            get_U(),
            get_H(),
            get_K()
        ]
    )

    # generate the trajectory
    time_steps, poses, velocities, accelerations = cuhk.get_trajectory()
    
    # optional: visualize the trajectory in top-down view
    get_2d_visualization(cuhk, "cuhk_topdown.png")
    
    # adjust the trajectory to fit the robot's workspace
    rotation_matrix = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    poses = np.dot(poses, rotation_matrix.T)  # (N, 3)
    poses = poses + np.array([500, 200, 100])
    # in our case, the orientation is fixed
    orientation = np.array(
        [[0, 0, 1],
        [1, 0, 0],
        [0, -1, 0]]
    )
    
    # solve ik for each pose
    all_thetas = []
    prev_thetas = np.zeros(6)  # initial joint angles
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

    # convert to degrees
    all_thetas = np.rad2deg(all_thetas)  
    
    # save the trajectory to a file
    out_string = "angle;"
    for j in range(6):
        for v in all_thetas[:, j]:
            out_string += f"{float(v)},"
        out_string = out_string[:-1] + ";"
    with open("joint_space.txt", "w") as f:
        f.write(out_string)
    
    # send the trajectory to the robot
    server_ip = "localhost" 
    server_port = 5000  # Replace with the server's port
    for i in range(all_thetas.shape[0]):
        angle_list = all_thetas[i, :].tolist()
        send_command(server_ip, server_port, angle_list)
        time.sleep(0.02)