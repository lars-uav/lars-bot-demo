import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(String, 'pose_topic', 10)
        self.timer = self.create_timer(5.0, self.publish_pose)

    def publish_pose(self):
        msg = String()
        # Format: curr_x curr_y heading goal_x goal_y
        msg.data = '0 0 U 2 3'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

