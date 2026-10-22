# Yang Lab AI Robot Beta Code Workplace

## Current Robot Status

Robot is able to move with prerouted (either via continuous recording or point teaching) trajectory. 
Achievable due to downloadable files from [Gitee Robot Arm](https://gitee.com/majormj/robot-arm-4340).
- `robot_arm_gripper_code\main.py` is main location to run robot arm routing
    - file also has capability to route grippers along with routing joints.

## New additions (not added by Gitee Robot Code)
The `camera_*_dector` files focus entirely on the camera and holds the original code of the cameras before modified to be incoporated into the Robot Arm application

### `robot_arm_gripper_code`
Added camera detection capability
