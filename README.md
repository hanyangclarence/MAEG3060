# MAEG 3060 Project

## Project Overview

We write the letter "CUHK" in this project. Each letter consists of a list of stroke, and for each stroke we define several key points, and use trajectory generation to complete the stroke. Between strokes an additional lift action is added.

For trajectory generation, we use cubic spline because it is more suitable for our task that requires curved and smooth connection between key points. We also implemented parabolic blend, but the generated trajectory is unreasonable. You can change the generation method in `classes/base_letters.py` line 29-42. The top-down visualization of the trajectory will be saved as `cuhk_topdown.png` after each run.

For inverse kinematics, we just implement the normal IK according to slides and the robot's configurations.

## Run code
No special dependencies is required. A general conda base environment is enough.

Just run:
```bash
python main.py
```
then the trajectory will be saved to `joint_space.txt`. Also, the program will keep sending signals to the local host. Press ctrl+C to stop it. 