# MAEG 3060 Project

## Project Overview

We write the letter "CUHK" in this project. Each letter consists of a list of stroke, and for each stroke we define several key points, and use trajectory generation to complete the stroke. Between strokes an additional lift action is added.

For trajectory generation, we apply cubic spline because it is more suitable for our task, which requires curved and smooth connections between key points. Linear trajectory with parabolic blends (LTPB) approach is also possible to implement; the generation method can be changed in `classes/base_letters.py` line 29-42. The top-down visualization of the trajectory will be saved as `cuhk_topdown.png` after each run.

For inverse kinematics, we just implement analytical IK according to slides and the robot's correct DH configurations.

## Run code
No special dependencies is required. A general conda base environment is enough.

Just run:
```bash
python main.py
```
then the trajectory will be saved to `joint_space.txt`. Also, the program will keep sending signals to the local host. Press ctrl+C to stop it. 
