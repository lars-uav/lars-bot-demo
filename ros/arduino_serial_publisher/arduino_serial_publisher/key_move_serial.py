# arduino_serial_publisher/key_move_serial.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .automation_optimised import automate_inputs
from .constants import *
import serial
import time

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            String,
            'pose_topic',
            self.listener_callback,
            10)
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)

    def front_grid(self):
        self.arduino.write(b'F')
        time.sleep(3.38)
        self.arduino.write(b'S')

    def back_grid(self):
        self.arduino.write(b'B')
        time.sleep(3.36)
        self.arduino.write(b'S')

    def right_grid(self):
        self.arduino.write(b'R')
        time.sleep(2.15)
        self.arduino.write(b'S')
        time.sleep(0.2)
        self.arduino.write(b'F')
        time.sleep(0.2)
        self.arduino.write(b'S')

    def listener_callback(self, msg):
        try:
            tokens = msg.data.strip().split()
            curr_x, curr_y = int(tokens[0]), int(tokens[1])
            heading = tokens[2]
            goal_x, goal_y = int(tokens[3]), int(tokens[4])

            self.get_logger().info(f"Received pose string: {msg.data}")
            poses, _ = automate_inputs(((curr_x, curr_y), heading.lower()), (goal_x, goal_y), (GRID_X, GRID_Y))
            self.get_logger().info(f"Determined Actions: {poses}")

            for action in poses:
                self.get_logger().info(f"Taking action: {action}")
                if action == 'f':
                    self.front_grid()
                elif action == 'r':
                    self.right_grid()
                elif action == 'b':
                    self.back_grid()
                time.sleep(1)

        except Exception as e:
            self.get_logger().error(f"Failed to parse or execute: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    node.arduino.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

