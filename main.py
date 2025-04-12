from classes.letters import *
from classes.base_letters import String, Letter, Stroke, get_2d_visualization

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
    print(f"Time steps: {time_steps.shape}")
    print(f"Poses: {poses.shape}")
    print(f"Velocities: {velocities.shape}")
    print(f"Accelerations: {accelerations.shape}")

    get_2d_visualization(cuhk, "cuhk.png")