import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from automation_optimised import automate_inputs
from constants import *
import time

class PoseDebugSubscriber(Node):
    def __init__(self):
        super().__init__('pose_debug_subscriber')
        self.subscription = self.create_subscription(
            String,
            'pose_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        try:
            tokens = msg.data.strip().split()
            curr_x, curr_y = int(tokens[0]), int(tokens[1])
            heading = tokens[2]
            goal_x, goal_y = int(tokens[3]), int(tokens[4])

            self.get_logger().info(f"Received pose string: {msg.data}")
            poses, _ = automate_inputs(((curr_x, curr_y), heading), (goal_x, goal_y), (GRID_X, GRID_Y))

            for action in poses:
                if action == 'f':
                    self.get_logger().info("Command: Move Forward")
                elif action == 'r':
                    self.get_logger().info("Command: Turn Right and Move")
                elif action == 'b':
                    self.get_logger().info("Command: Move Backward")
                elif action == 'l':
                    self.get_logger().info("Command: Turn Left and Move")
                time.sleep(1)

        except Exception as e:
            self.get_logger().error(f"Failed to parse or simulate: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseDebugSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

