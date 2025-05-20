import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

import math
import numpy as np
import cv2
import time
import yaml
from pathlib import Path
from multiprocessing import shared_memory, Queue, Event

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessorLegacy as VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig
from pytransform3d import rotations


class DisplacementPublisher(Node):
  def __init__(self):
      super().__init__('displacement_publisher')
      self.publisher = self.create_publisher(Float32MultiArray, '/displacement', 10)
      self.publisher2 = self.create_publisher(Float32, '/finger', 10)
      self.timer = self.create_timer(1/120, self.publish_displacement)
      self.timer_finger = self.create_timer(1/120, self.publish_finger)

  def publish_displacement(self, displacement):
        msg = Float32MultiArray()
        msg.data = displacement
        self.publisher.publish(msg)
        self.get_logger().info(f'Published displacement: {displacement}')
  def publish_finger(self,distance):
    msg = Float32()
    msg.data = distance
    self.publisher2.publish(msg)
    self.get_logger().info(f'Published left finger data: {distance}')


class USBCameraSystem:
    def __init__(self, camera_id=0, resolution=(720, 1280)):
        self.resolution = resolution
        self.cam = cv2.VideoCapture(camera_id)

        if not self.cam.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_id}")

        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[1])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[0])

    def get_frames(self):
        ret, frame = self.cam.read()
        if not ret:
            print("Failed to capture frame from camera")
            return None, None

        if frame.shape[:2] != self.resolution:
            frame = cv2.resize(frame, (self.resolution[1], self.resolution[0]))

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame.copy(), frame.copy()

    def release(self):
        self.cam.release()


class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()

    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        head_rmat = head_mat[:3, :3]

        left_pose = np.concatenate([left_wrist_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
                                    rotations.quaternion_from_matrix(left_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
        right_pose = np.concatenate([right_wrist_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
                                     rotations.quaternion_from_matrix(right_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        # thumb_tip = left_qpos[4]
        # index_tip = left_qpos[9]
        # distance = float(np.linalg.norm(thumb_tip - index_tip))
        thumb_tip = left_hand_mat[tip_indices][0]  # Thumb tip (3D position)
        index_tip = left_hand_mat[tip_indices][1]  # Index finger tip (3D position)
        distance = float(np.linalg.norm(thumb_tip - index_tip))
        #print("distance:", distance)
        return head_rmat, left_pose, right_pose, left_qpos, right_qpos, distance

class Sim:
    def __init__(self, print_freq=True):
        rclpy.init()
        self.publisher = DisplacementPublisher()
        self.print_freq = print_freq
        self.target_pose = np.array([-0.4, 0.08, 1.1]) 
        self.threshold = 0.1
        self.camera_system = USBCameraSystem()
        self.active_position = None
        self.active_orientation = None
        self.status = "Inactive"
        self.displacement = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.left_finger = np.zeros(12)
        self.previous_pose = np.array([-0.45, 0.25,1.6]) #np.array([-0.45, 0.16, 1.34])
        self.distance = 0.0
        #new
        #self.gripper_threshold = 0.5


    def step(self, head_rmat, left_pose, right_pose, left_qpos, left_finger, distance):
        if self.print_freq:
            start = time.time()
        #print("left_pose",left_pose)
        # 使用字符串格式化来打印
        #print("Formatted left_qpos:", ", ".join([f"{q:.2f}" for q in left_qpos]))
        #print("left_finger:", self.left_finger)

        left_position = np.array(left_pose[:3])
        left_orientation = np.array(left_pose[3:6])
        # left_orientation[0] -= 90  # 偏移 roll 90 度，讓手掌方向對應到夾爪朝下
        self.distance = distance

        if self.status == "Inactive":
            if np.all(np.abs(left_position - self.target_pose) <= self.threshold):
                self.status = "Active"
                if self.active_position is None:
                    self.active_position = left_position
                    self.active_orientation = np.array([ -0.45, 0, left_orientation[2]])    
                self.previous_pose = left_position  
        else:
            if self.previous_pose is not None:
                displacement_posi = left_position - self.active_position 
                left_orientation = np.array([left_orientation[1], left_orientation[0], left_orientation[2]])
                displacement_orien = left_orientation - self.active_orientation
                self.displacement = np.concatenate(( displacement_posi, displacement_orien))
                self.left_finger = np.array(left_qpos)

            else:
                self.displacement = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                
            self.previous_pose = left_position
            self.publisher.publish_displacement(self.displacement.tolist())
            self.publisher.publish_finger(self.distance)

        left_image, right_image = self.camera_system.get_frames()
        if left_image is None or right_image is None:
            left_image = np.zeros((self.camera_system.resolution[0], self.camera_system.resolution[1], 3), dtype=np.uint8)
            right_image = np.zeros((self.camera_system.resolution[0], self.camera_system.resolution[1], 3), dtype=np.uint8)
        else:
            cv2.putText(left_image, f"Status: {self.status}", (400, 250), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            left_text = [
                f"Left: x={left_pose[0]:.2f}",
                f"y={left_pose[1]:.2f}",
                f"z={left_pose[2]:.2f}",
                # f"index={left_qpos[0]:.2f}",
                # f"thumb={left_qpos[6]:.2f}",
                f"width={distance:.2f}"
                # f"displacement={self.displacement}"
            ]
            for i, line in enumerate(left_text):
                cv2.putText(left_image, line, (400, 300 + i * 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
            if self.status == "Active":
                displacement_text = [
                    f"dx={self.displacement[0]:.2f}",
                    f"dy={self.displacement[1]:.2f}",
                    f"dz={self.displacement[2]:.2f}",
                    f"roll={self.displacement[3]:.2f}",
                    f"pitch={self.displacement[4]:.2f}",
                    f"yaw={self.displacement[5]:.2f}"
                ]
                #print("dispace", displacement_text)
                # for i, line in enumerate(displacement_text):
                #     cv2.putText(right_image, line, (600, 300 + i * 50), 
                #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                right_text = [
                    f"Left: roll={left_pose[4] + 0.45:.2f}",
                    f"pitch={left_pose[3] :.2f}",
                    f"yaw={left_pose[5]:.2f}"
                ]
                # right_text = [
                #     f"Right: x={right_pose[0]:.2f}",
                #     f"y={right_pose[1]:.2f}",
                #     f"z={right_pose[2]:.2f}"
                # ]
                for i, line in enumerate(right_text):
                    cv2.putText(right_image, line, (600, 300 + i * 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            left_image = cv2.cvtColor(cv2.cvtColor(left_image, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2RGB)
            right_image = cv2.cvtColor(cv2.cvtColor(right_image, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2RGB)

        if self.print_freq:
            end = time.time()
            #print('Frequency:', 1 / (end - start))

        #print("Returning left_finger:", self.left_finger)
        return left_image, right_image

    def end(self):
        self.camera_system.release()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        teleoperator = VuerTeleop('inspire_hand.yml')
        simulator = Sim()

        test_frame = simulator.camera_system.get_frames()[0]
        if test_frame is None:
            raise RuntimeError("Camera initialization failed")

        while True:
            head_rmat, left_pose, right_pose, left_qpos, right_qpos, distance = teleoperator.step()
            left_img, right_img = simulator.step(head_rmat, left_pose, right_pose, left_qpos, right_qpos, distance)
            #left_img, right_img = simulator.step(head_rmat, left_pose, right_pose, left_qpos, right_qpos)
            result = simulator.step(head_rmat, left_pose, right_pose, left_qpos, right_qpos, distance)

            if left_img is not None and right_img is not None:
                np.copyto(teleoperator.img_array, np.hstack((left_img, right_img)))

    except KeyboardInterrupt:
        simulator.end()
        exit(0)
    except Exception as e:
        print(f"Error occurred: {e}")
        simulator.end()
        exit(1)