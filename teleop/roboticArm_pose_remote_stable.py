import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Float32
from argparse import ArgumentParser
from frankx import *
from time import sleep
import numpy as np
from scipy.spatial.transform import Rotation as R

#Calculate eef pose and quat
#==========# 
def matrix_to_quaternion(matrix):
    rot = R.from_matrix(np.array(matrix).reshape(4, 4)[:3, :3])
    return rot.as_quat().tolist()  # [x, y, z, w]

def extract_translation(matrix):
    mat = np.array(matrix).reshape((4, 4), order='F')  # 'F' means column-major
    return mat[0:3, 3].tolist()
#==========# 

class DisplacementSubscriber(Node):
    def __init__(self, robot):
        super().__init__('displacement_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/displacement',
            self.listener_callback,
            10)
        self.subscription

        self.finger_subscription = self.create_subscription(
            Float32,
            '/finger',
            self.listener_callback_finger,
            10)
        self.subscription

        self.robot = robot  
        self.delta_range = 0.05  
        self.initial_position = [0.4, 0.1, 0.07] #[0.3, 0, 0.5](un-comment the block below to see the eef pos and quat )
        
        self.left_qpos = 0.0  
        self.last_gripper_state = None
        self.is_clamp = False

        self.gripper = self.robot.get_gripper()
        self.gripper.homing()#gripper init 

        #joint_motion = JointMotion([0.0053, -0.312 ,  0.2176, -2.5857,  0.0787,  2.2788,  1.8525])
        #joint_motion = JointMotion([-0.0, -0.796, 0.0, -2.329, 0.0, 1.53, 0.785])
        joint_motion = JointMotion([0.0392,  0.1765,  0.1852, -2.7013, -0.1484,  2.8729,  1.9007])
        robot.move(joint_motion)

        # Read eef pos, quat
        #==========# 
        # pose = robot.read_once()
        # robot0_eef_pos = extract_translation(pose.O_T_EE)
        # robot0_eef_quat = matrix_to_quaternion(pose.O_T_EE)
        # print("Current EEF position:", robot0_eef_pos)
        # print("Current EEF quaternion:", robot0_eef_quat)
        #==========# 

        #self.gripper = self.robot.get_gripper()
        self.impedance_motion = ImpedanceMotion(200.0, 20.0) 
        self.robot_thread = self.robot.move_async(self.impedance_motion) 
        sleep(0.1)

        self.initial_target = self.impedance_motion.target
        self.get_logger().info(f'Initial target: {self.initial_target}')

    def listener_callback_finger(self, msg):
        if msg.data != 0:
            new_qpos = msg.data
            print("left_qpos:", self.left_qpos)
            if abs(new_qpos - self.left_qpos) > 0.055:  # Only act on significant change
                self.left_qpos = new_qpos
                self.get_logger().info(f'Finger data received: {self.left_qpos}')

                # Gripper open status logic
                scale = 0.4
                self.get_logger().info(f'Processed finger data received: {self.left_qpos * scale}')
                threshold = 0.04 #0.04
                should_open = (self.left_qpos * scale > threshold)

                from threading import Thread
                import time
                
                if should_open :    
                    print("width:", self.left_qpos * scale)
                    #gripper_width = 0.06
                    #self.gripper.release(0.06)
                    #Thread(target=self.gripper.move_async, args=(gripper_width,)).start()
                    Thread(target=self.gripper.open).start()
                    self.is_clamp = False
                else :
                    if not self.is_clamp:
                        Thread(target=self.gripper.clamp).start()
                        self.is_clamp = True
                # Move gripper in a separate thread so it doesn't block Switch to move_unsafe_async might helps?
                #gripper_width = self.left_qpos * 0.5
                #from threading import Thread
                print("width:", self.left_qpos * scale)
                #Thread(target=self.gripper.move_async, args=(gripper_width,)).start()
                #Thread(target=self.gripper.move_async, args=(self.left_qpos * 0.45,)).start()
                
        else:
            self.get_logger().warn('Received finger does not have values.')

    def listener_callback(self, msg):
        displacement = msg.data
        if len(displacement) == 6:
            x, y, z, roll, pitch, yaw= displacement
            self.apply_relative_motion( x, y, z, pitch, yaw)
        else:
            self.get_logger().warn('Received displacement does not have exactly 3 values.')

    def apply_relative_motion(self, delta_x, delta_y, delta_z, pitch, yaw):
        # delta_x = 0
        # delta_y = 0
        # delta_z = 0

        new_x = self.initial_position[0] + delta_x * 1.3
        new_y = self.initial_position[1] + delta_y * 1.3
        new_z = self.initial_position[2] + delta_z
        # Convert units
        pitch = pitch / 10 * 17.4
        yaw = yaw / 10 * 17.4

        if not (0.25 <= new_x <= 0.66):
            self.get_logger().warn(f'X value {new_x:.4f} out of range (0.25 - 0.56). Motion ignored.')
            return
        if not (-0.55 <= new_y <= 0.55):
            self.get_logger().warn(f'Y value {new_y:.4f} out of range (-0.35 - 0.34). Motion ignored.')
            return
        if not (new_z <= 0.71):
            self.get_logger().warn(f'Z value {new_z:.4f} out of range (must be <= 0.71). Motion ignored.')
            return

        if hasattr(self, 'impedance_motion'):
            self.impedance_motion.target = Affine(new_x, new_y, new_z, yaw, 0, 0)
            sleep(1/120)
            #self.get_logger().info(f'Robot moved: dx={new_x:.4f}, dy={new_y:.4f}, dz={new_z:.4f}, roll=0, pitch={pitch}')
        else:
            self.get_logger().error('Impedance motion is not initialized!')


    def stop_motion(self):
        if hasattr(self, 'impedance_motion'):
            self.impedance_motion.finish()
            self.robot_thread.join()
            self.get_logger().info("Impedance motion finished.")

def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    robot = Robot(args.host)

    robot.set_default_behavior()
    robot.recover_from_errors()
    robot.set_dynamic_rel(0.15) 
    
    subscriber = DisplacementSubscriber(robot)
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.stop_motion() 
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()