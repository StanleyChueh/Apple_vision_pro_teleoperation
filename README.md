# 🖐️ Apple Vision Pro Teleoperation for Franka Emika Panda

This project enables intuitive teleoperation of a **Franka Emika Panda** robotic arm using **Apple Vision Pro** hand tracking, integrating [OpenTeleVision](https://github.com/unitreerobotics/avp_teleoperate.git) with the low-level **Frankx** control framework.

Inspired by Unitree Robotics' original [AVP Teleoperate](https://github.com/unitreerobotics/avp_teleoperate.git), this implementation allows real-time control of the robot arm using wrist pose and finger gesture recognition.

---

## 🚀 Features

- 🎯 Real-time teleoperation using Apple Vision Pro
- 🧠 Low-latency robot control via `Frankx`
- ✋ Hand gesture tracking: wrist pose and thumb–index pinch distance
- 🔁 ROS 2 integration with two custom topics:
  - `/finger`: thumb–index finger distance
  - `/displacement`: wrist pose delta
- 🤖 Seamless retargeting to Franka Panda's wrist joint

---

## 📦 Prerequisites

- Ubuntu 20.04+
- ROS 2 (Foxy or later)
- Python 3.8+
- [Frankx](https://github.com/pantor/frankx.git)
- [OpenTeleVision](https://github.com/unitreerobotics/avp_teleoperate.git)(h1 branch)
- [Franka Emika Panda](https://franka.de/)

---

## 🧪 How to Run

Open **Terminal 1** to launch hand tracking input:
```bash
python3 teleop_arm_pose_stable.py
```
> Publishes `/finger` and `/displacement` topics based on Vision Pro hand pose and gesture tracking.

Then, open **Terminal 2** to control the robot:
```bash
python3 roboticArm_pose_remote_stable.py
```
> Subscribes to the above topics and sends real-time commands to the Franka Emika Panda using `Frankx`.

---

## 🎥 Demo

https://youtube.com/shorts/8B0iJiU9TVg?feature=share

![demo_gif](https://github.com/user-attachments/assets/9849d4c4-e391-431a-b1c5-16a4aa3da3c2)

---

## 📁 Repository Structure 

```bash

├──avp_teleoperate
  ├──act
  ├──assets
  ├──img
  ├──scripts
  ├──teleop
    ├── [teleop_arm_pose_stable.py]         # Publishes hand tracking data
    ├── [roboticArm_pose_remote_stable.py]  # Controls the Franka arm
    ├── README.md
```
##### Please make sure you have set up [OpenTelevision environment](https://github.com/unitreerobotics/avp_teleoperate.git)
---

## 📝 License

This project is released under the MIT License.
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
