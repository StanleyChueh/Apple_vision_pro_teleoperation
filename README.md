# Apple_vision_pro_teleoperation
## Introduction
This repository is inspired by Unitree Robotics' avp_teleoperate: https://github.com/unitreerobotics/avp_teleoperate.git. 

We integrate the OpenTeleVision component with the Franka Emika Panda robot using the Frankx low-level control framework.
## How to run it
Open one terminal and run:
```
python3 teleop_arm_pose_stable.py
```
This code publishes two ROS 2 topics: one for the distance between the index finger and thumb (/finger), and another for wrist pose displacement (/displacement). It can retarget the user's wrist pose to the robotic arm's wrist joint.

Open another terminal and run:
```
python3 roboticArm_pose_remote_stable.py
```
This code integrates the /finger and /displacement topics, and uses Frankx to control a real Franka Emika Panda robot.

![image](https://github.com/user-attachments/assets/9849d4c4-e391-431a-b1c5-16a4aa3da3c2)

Demo: https://youtube.com/shorts/8B0iJiU9TVg?feature=share
