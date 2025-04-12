import numpy as np
import typing as tp
import matplotlib.pyplot as plt

from libs.trajectory_generation import generate_trajectory_3d
from const import *

# Note: By default, x is the horizontal axis, y is the vertical axis, and z is the height


class Stroke:
    def __init__(
        self,
        poses: tp.List[np.ndarray],
        time_list: tp.List[float],
    ):
        '''
        This class implements a stroke, which is a series of poses that the end effector can draw in one pass without the need to lift the pen.
        '''
        self.poses = np.array(poses)  # List of poses (x, y, z)
        # validate the poses, should not exceeds the bounding box
        assert all(isinstance(pose, np.ndarray) and pose.shape == (3,) for pose in poses), "All poses must be 3D numpy arrays"
        assert all(0 <= pose[0] <= MAX_WIDTH_HEIGHT and 0 <= pose[1] <= MAX_WIDTH_HEIGHT and pose[2] == 0 for pose in poses), "All poses must be within the specified bounds"
        self.time_list = time_list  # List of time points for each pose
        assert len(poses) == len(time_list), "Length of poses and time_list must be the same"
        self.duration = time_list[-1] - time_list[0]  # the total time to write the stroke

        # solve the trajectory using cubic polynomial
        self.time_steps, self.poses, self.velocities, self.accelerations = generate_trajectory_3d(
            self.poses,
            self.time_list,
            frequency=50,
        )
    
    def get_trajectory(self) -> tp.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get the trajectory of the stroke.
        """
        return self.time_steps, self.poses, self.velocities, self.accelerations


class Lift(Stroke):
    def __init__(
        self,
        start_pose: np.ndarray,
        end_pose: np.ndarray,
        move_duration: float,
        lift_height: float = MAX_LIFT_HEIGHT,
        lift_duration: float = LIFT_DURATION,
    ):
        """
        This is the motion of lifting the pen to move to a new position.
        It acts as a connection between two strokes (if they are not connected, like the strokes to draw 'H').
        """
        start_intermediate = start_pose + np.array([0, 0, lift_height])
        end_intermediate = end_pose + np.array([0, 0, lift_height])
        poses = [start_pose, start_intermediate, end_intermediate, end_pose]
        time_list = [0, lift_duration, lift_duration + move_duration, lift_duration + move_duration + lift_duration]
        super().__init__(poses, time_list)


class Letter:
    def __init__(
        self, 
        letter: str,
        strokes: tp.List[Stroke],
    ):
        """
        The class of a letter, consists of a list of strokes.
        Here in the initialization, we automatically add the lift aftion between strokes if they are not connected.
        """
        assert len(letter) == 1, "Letter must be a single character"
        self.letter = letter

        # add lift between strokes
        moves = []
        for i in range(len(strokes) - 1):
            moves.append(strokes[i])
            if strokes[i].poses[-1] != strokes[i + 1].poses[0]:
                # if the two strokes are not connected, add a lift stroke
                lift = Lift(
                    strokes[i].poses[-1],
                    strokes[i + 1].poses[0],
                    move_duration=1.0,  # TODO: make this a more reasonable value, maybe change with the move distance
                )
                moves.append(lift)
        moves.append(strokes[-1])

        # validate the moves: they must be connected
        for i, move in enumerate(moves):
            if i != 0:
                assert move.poses[0] == moves[i - 1].poses[-1], f"Stroke {i} must start where stroke {i-1} ends"
            if i != len(moves) - 1:
                assert move.poses[-1] == moves[i + 1].poses[0], f"Stroke {i} must end where stroke {i+1} starts"
        
        self.moves = moves

        self.duration = sum(move.duration for move in moves)  # the total time to write the letter
    
    def get_trajectory(self) -> tp.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get the trajectory of the letter by concatenating the trajectories of all strokes.
        """
        all_time_steps = []
        all_poses = []
        all_velocities = []
        all_accelerations = []

        global_time = 0
        for move in self.moves:
            # shift the time steps to start from the end of the previous move
            time_steps = move.time_steps + global_time
            all_time_steps.append(time_steps)
            all_poses.append(move.poses)
            all_velocities.append(move.velocities)
            all_accelerations.append(move.accelerations)

            global_time += move.duration
        
        # concatenate all the trajectories
        all_time_steps = np.concatenate(all_time_steps)
        all_poses = np.concatenate(all_poses)
        all_velocities = np.concatenate(all_velocities)
        all_accelerations = np.concatenate(all_accelerations)
        return all_time_steps, all_poses, all_velocities, all_accelerations
            

class String:
    def __init__(
        self,
        letters: tp.List[Letter],
        letter_width: float = MAX_WIDTH_HEIGHT,
    ):
        """
        String is a collection of letters
        """
        self.letters = letters
        self.letter_width = letter_width  # the width of the letter
        
    def get_trajectory(self) -> tp.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get the global trajectory of the string by concatenating the trajectories of all letters.
        """
        all_time_steps = []
        all_poses = []
        all_velocities = []
        all_accelerations = []

        global_time = 0
        for idx, letter in enumerate(self.letters):
            pose_offset = np.array([idx * self.letter_width, 0, 0])
            time_steps, poses, velocities, accelerations = letter.get_trajectory()

            all_time_steps.append(time_steps + global_time)  # shift the time steps to start from the end of the previous letter
            all_poses.append(poses + pose_offset)  # shift the poses to the right for each letter
            all_velocities.append(velocities)
            all_accelerations.append(accelerations)

            global_time += letter.duration

            # add a lift stroke between letters
            if idx != len(self.letters) - 1:
                lift = Lift(
                    start_pose=poses[-1],
                    end_pose=self.letters[idx + 1].moves[0].poses[0] + np.array([self.letter_width, 0, 0]),
                    move_duration=1.0,  # TODO: make this a more reasonable value, maybe change with the move distance
                )
                all_time_steps.append(lift.time_list + global_time)
                all_poses.append(lift.poses + pose_offset)
                all_velocities.append(lift.velocities)
                all_accelerations.append(lift.accelerations)

                global_time += lift.duration
        
        # concatenate all the trajectories
        all_time_steps = np.concatenate(all_time_steps)
        all_poses = np.concatenate(all_poses)
        all_velocities = np.concatenate(all_velocities)
        all_accelerations = np.concatenate(all_accelerations)
        return all_time_steps, all_poses, all_velocities, all_accelerations
    
        
def get_2d_visualization(obj, save_filename: str):
    """
    Get the 2D visualization of the object.
    This function first get the poses from the object (could be Stroke, String or Letter),
    then remove the poses with non-zero z value.
    The remaining poses are then plotted in 2D.
    """
    assert hasattr(obj, "get_trajectory"), "Object must have a get_trajectory method"
    _, poses, _, _ = obj.get_trajectory()  # poses: (T, 3)
    
    # split the poses by sections of non-zero z value
    sections = []
    sec_start = 0
    while sec_start < len(poses):
        if poses[sec_start][2] == 0:
            sec_start += 1
            continue
        sec_end = sec_start + 1
        while sec_end < len(poses) and poses[sec_end][2] != 0:
            sec_end += 1
        sections.append(poses[sec_start:sec_end])
        sec_start = sec_end
    
    # plot the sections
    if isinstance(obj, String):
        plt.figure(figsize=(10 * len(obj.letters), 10))
    else:
        plt.figure(figsize=(10, 10))
    for i, section in enumerate(sections):
        plt.plot(section[:, 0], section[:, 1], label=f"Section {i + 1}")
        plt.scatter(section[:, 0], section[:, 1], s=10, c="red")
    plt.xlabel("X")
    plt.ylabel("Y")

    plt.savefig(save_filename)
    plt.close()

