# 🖐️ Apple Vision Pro Teleoperation for Franka Emika Panda 

This branch aims to use vision pro to control franka for data collection in LeRobot, so we will take you to walk through the pipeline, and the reason to switch control framework from frankx to franka_ros

---

## 🚀 Features (Compared to `main` Branch)

- ✅ Enables reading robot states during motion execution
- 📈 Increases robot state frequency from ~17 Hz to **40+ Hz**
- 📝 Adds recording functionality for LeRobot dataset format

---

## 📦 Prerequisites

- Ubuntu **20.04**
- ROS 1 **Noetic**
- Python **3.8+**
- [franka_ros](https://github.com/frankaemika/franka_ros.git)
- [OpenTeleVision (h1 branch)](https://github.com/unitreerobotics/avp_teleoperate.git)
- [Franka Emika Panda](https://franka.de/)

---

## 🧪 How to Run

### 1️⃣ Launch `franka_ros`
```bash
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true launch_rviz:=false 
```
> Launches Franka control in joint position mode, with impedance control loaded (but inactive).

### 2️⃣ Start Robot Arm Control
```bash
python roboticArm_pose_remote_ros1.py
```
> Subscribes to ROS topics and sends real-time control commands to Franka using franka_ros.

### 3️⃣ Start Vision Pro Hand Tracking Input
```bash
python teleop_arm_pose_ros1.py
```
> Publishes `/finger` and `/displacement` topics based on Vision Pro hand pose and gesture tracking.

### 4️⃣ (Optional) Record Data in LeRobot Format
```bash
python record_ros1.py --repo_id test --single_task test 
```
> Use gestures or keys to save/discard episodes:
> 
>   ✅ Right-hand pinch (thumb–index) → Save
> 
>   ❌ Both-hand pinch → Discard

---

## 🎥 Demo

https://youtube.com/shorts/8B0iJiU9TVg?feature=share

![demo_gif](https://github.com/user-attachments/assets/9849d4c4-e391-431a-b1c5-16a4aa3da3c2)

---

## 📁 Repository Structure 

```bash

avp_teleoperate/
├── act/
├── assets/
├── img/
├── scripts/
├── teleop/
│   ├── teleop_arm_pose_stable.py           # Hand tracking (main branch)
│   ├── roboticArm_pose_remote_stable.py    # Robot control (main branch)
│   ├── teleop_arm_pose_ros1.py             # Hand tracking (ROS1/dev)
│   ├── roboticArm_pose_remote_ros1.py      # Robot control (ROS1/dev)

----

franka_ros/
└── src/
    └── franka_ros/
        └── franka_example_controllers/
            └── launch/
                └── cartesian_impedance_example_controller.launch  # Modified launch file
----

franka_record/
├── franka_dataset.py                       # LeRobot-compatible dataset structure
├── record_ros1.py                          # Recorder script

```
## 🔧 Modified Launch File

To enable joint position control and load impedance control in stopped mode, use the following version of cartesian_impedance_example_controller.launch:

```
<?xml version="1.0" ?>
<launch>
  <arg name="robot" default="panda" doc="choose your robot. Possible values: [panda, fr3]"/>
  <arg name="arm_id" default="$(arg robot)" />
  <arg name="launch_rviz" default="true" doc="Whether to launch RViz" />
  <arg name="robot_ip" default="172.16.0.2" />
  <include file="$(find franka_control)/launch/franka_control.launch" pass_all_args="true"/>
  <rosparam command="load" file="$(find franka_example_controllers)/config/franka_example_controllers.yaml" subst_value="true" />
  <node name="joint_controller_spawner" pkg="controller_manager" type="spawner" output="screen" args="position_joint_trajectory_controller" />
  <node name="impedance_controller_spawner" pkg="controller_manager" type="spawner" output="screen" args="cartesian_impedance_example_controller --stopped" />
  <node if="$(arg launch_rviz)" pkg="rviz" type="rviz" output="screen" name="rviz" args="-d $(find franka_example_controllers)/launch/rviz/franka_description_with_marker.rviz"/>
  <node name="rqt_reconfigure" pkg="rqt_reconfigure" type="rqt_reconfigure" required="false" />
</launch>
```
##### Please ensure both OpenTeleVision and franka_ros are properly installed and sourced.
---

## 📝 License

#### This project is released under the MIT License.
---

## 🤝 Acknowledgments

- [Unitree Robotics - avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate.git)
- [pantor - Frankx](https://github.com/pantor/frankx.git)

### ⚠️ Known Issues

This project was originally developed to control a **Franka Emika Panda** robot using **Apple Vision Pro**, aiming to collect datasets in the [LeRobot](https://github.com/huggingface/lerobot.git) format for training **ACT** models. However, the following limitations with the `frankx` control library were identified:

1. ❌ Cannot read robot state while executing motion commands.
2. 🐢 Low robot state update rate (~15–17 Hz), whereas **LeRobot** ideally requires ≥ 30 Hz.

Due to these constraints, we have transitioned to using the [**franka_ros**](https://github.com/frankarobotics/franka_ros.git) framework as the low-level control layer in subsequent development.

👉 **For details on the updated implementation, please check the [`dev`](https://github.com/StanleyChueh/Apple_vision_pro_teleoperation.git/tree/dev) branch.**
