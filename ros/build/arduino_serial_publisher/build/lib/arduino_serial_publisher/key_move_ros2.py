#!/usr/bin/env python3

import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from automation_optimised import automate_inputs
from constants import *

# Serial setup
arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# Global variables to store pose info across messages
current_pose = None
final_pose = None

def bytes_pose_to_tuple(b: str):
    b = b.strip().split(" ")
    b[0] = int(b[0])
    b[1] = int(b[1])
    return b

# Movement commands from original key_move
def front_grid():
    arduino.write(b'F')
    time.sleep(3.38)
    arduino.write(b'S')

def back_grid():
    arduino.write(b'B')
    time.sleep(3.36)
    arduino.write(b'S')

def right_grid():
    arduino.write(b'R')
    time.sleep(2.15)
    arduino.write(b'S')
    time.sleep(0.2)
    arduino.write(b'F')
    time.sleep(0.2)
    arduino.write(b'S')

def left_grid():
    arduino.write(b'L')
    time.sleep(2)
    arduino.write(b'S')

class KeyMoveROS2(Node):
    def __init__(self):
        super().__init__('key_move_ros2')
        self.subscription = self.create_subscription(
            String,
            'robot_input',
            self.input_callback,
            10
        )
        self.get_logger().info('ROS2 KeyMove Node Started')

    def input_callback(self, msg):
        global current_pose, final_pose
        char = msg.data.strip().lower()

        # Movement Keys
        if char == 'q':
            arduino.write(b'S')
            self.get_logger().info("Quitting... (q)")
            rclpy.shutdown()

        elif char in ['w', 'up']:
            arduino.write(b'F')
            self.get_logger().info("Move forward")

        elif char in ['s', 'down']:
            arduino.write(b'B')
            self.get_logger().info("Move backward")

        elif char in ['a', 'left']:
            arduino.write(b'L')
            self.get_logger().info("Turn left")

        elif char in ['d', 'right']:
            arduino.write(b'R')
            self.get_logger().info("Turn right")

        elif char in [' ', 'brake']:
            arduino.write(b'S')
            self.get_logger().info("Brake")

        # One-cell grid movements
        elif char == 'j':
            front_grid()
            self.get_logger().info("Front Grid")

        elif char == 'k':
            back_grid()
            self.get_logger().info("Back Grid")

        elif char == 'l':
            right_grid()
            self.get_logger().info("Right Grid")

        elif char == 'h':
            left_grid()
            self.get_logger().info("Left Grid")

        # Auto mode - multi-part input
        elif char.startswith("curr:"):
            try:
                current_pose = bytes_pose_to_tuple(char.replace("curr:", ""))
                self.get_logger().info(f"Set current pose: {current_pose}")
            except:
                self.get_logger().error("Invalid current pose format!")

        elif char.startswith("dest:"):
            try:
                final_pose = bytes_pose_to_tuple(char.replace("dest:", ""))
                self.get_logger().info(f"Set final pose: {final_pose}")
            except:
                self.get_logger().error("Invalid final pose format!")

        elif char == 'x':
            if current_pose and final_pose:
                try:
                    self.get_logger().info(f"Starting auto move: {current_pose} -> {final_pose}")
                    poses, _ = automate_inputs(((current_pose[0], current_pose[1]), current_pose[2]),
                                               (final_pose[0], final_pose[1]),
                                               (GRID_X, GRID_Y))
                    self.get_logger().info(f"Auto path: {poses}")

                    for pose in poses:
                        if pose == 'f':
                            front_grid()
                        elif pose == 'r':
                            right_grid()
                        elif pose == 'b':
                            back_grid()
                        time.sleep(1)
                except Exception as e:
                    self.get_logger().error(f"Auto move failed: {e}")
            else:
                self.get_logger().warn("Pose data missing. Send 'curr:x y θ' and 'dest:x y' first.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyMoveROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        arduino.close()

if __name__ == '__main__':
    main()

