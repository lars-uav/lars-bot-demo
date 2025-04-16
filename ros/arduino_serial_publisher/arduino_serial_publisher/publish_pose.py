import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(String, 'pose_topic', 10)
        self.current_pose = {'x': 0, 'y': 0, 'heading': 'U'}
        self.goal_pose = {'x': 0, 'y': 0}
        
    def publish_pose(self):
        msg = String()
        # Format: curr_x curr_y heading goal_x goal_y
        pose_str = f"{self.current_pose['x']} {self.current_pose['y']} {self.current_pose['heading']} {self.goal_pose['x']} {self.goal_pose['y']}"
        msg.data = pose_str
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def publish_movement(self, movement):
        msg = String()
        msg.data = movement
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        
    def bytes_pose_to_tuple(self, b: bytes):
        b = b.decode("utf-8").strip().split(" ")
        b[0] = int(b[0])
        b[1] = int(b[1])
        return b
        
    def front_grid(self):
        self.publish_movement('f')

        if self.current_pose['heading'] == 'U':
            self.current_pose['y'] += 1
        elif self.current_pose['heading'] == 'R':
            self.current_pose['x'] += 1
        elif self.current_pose['heading'] == 'D':
            self.current_pose['y'] -= 1
        elif self.current_pose['heading'] == 'L':
            self.current_pose['x'] -= 1
    
    def back_grid(self):
        self.publish_movement('b')

        if self.current_pose['heading'] == 'U':
            self.current_pose['y'] -= 1
        elif self.current_pose['heading'] == 'R':
            self.current_pose['x'] -= 1
        elif self.current_pose['heading'] == 'D':
            self.current_pose['y'] += 1
        elif self.current_pose['heading'] == 'L':
            self.current_pose['x'] += 1
    
    def right_grid(self):
        self.publish_movement('r')

        # Update heading for right turn
        if self.current_pose['heading'] == 'U':
            self.current_pose['heading'] = 'R'
        elif self.current_pose['heading'] == 'R':
            self.current_pose['heading'] = 'D'
        elif self.current_pose['heading'] == 'D':
            self.current_pose['heading'] = 'L'
        elif self.current_pose['heading'] == 'L':
            self.current_pose['heading'] = 'U'
    
    def left_grid(self):
        self.publish_movement('l')

        # Update heading for left turn
        if self.current_pose['heading'] == 'U':
            self.current_pose['heading'] = 'L'
        elif self.current_pose['heading'] == 'L':
            self.current_pose['heading'] = 'D'
        elif self.current_pose['heading'] == 'D':
            self.current_pose['heading'] = 'R'
        elif self.current_pose['heading'] == 'R':
            self.current_pose['heading'] = 'U'
    
    def set_current_pose(self, x, y, heading):
        self.current_pose['x'] = x
        self.current_pose['y'] = y
        self.current_pose['heading'] = heading
        self.publish_pose()
    
    def set_goal_pose(self, x, y):
        self.goal_pose['x'] = x
        self.goal_pose['y'] = y
        self.publish_pose()
        self.current_pose['x'] = x
        self.goal_pose['y'] = y
        
    def run_curses_interface(self):
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.keypad(True)
        
        try:
            while True:
                screen.clear()
                screen.addstr(0, 0, "ROS2 Pose Publisher. Press 'q' to quit.\n")
                screen.addstr("Press arrow keys to move the robot.\n")
                screen.addstr("Press x for autonomous grid movement.\n")
                screen.addstr("Current pose: x={}, y={}, heading={}\n".format(
                    self.current_pose['x'], self.current_pose['y'], self.current_pose['heading']))
                screen.addstr("Goal pose: x={}, y={}\n".format(
                    self.goal_pose['x'], self.goal_pose['y']))
                screen.refresh()
                
                char = screen.getch()
                
                if char == ord('q'):
                    break
                elif char == curses.KEY_UP or char == ord('w'):
                    self.front_grid()
                elif char == curses.KEY_DOWN or char == ord('s'):
                    self.back_grid()
                elif char == curses.KEY_LEFT or char == ord('a'):
                    self.left_grid()
                elif char == curses.KEY_RIGHT or char == ord('d'):
                    self.right_grid()
                elif char == ord('x'):
                    try:
                        screen.addstr('Enter current pose in format: x y theta(U/D/L/R) (matrix indices)\n')
                        curr_pose = self.bytes_pose_to_tuple(screen.getstr())
                        screen.addstr('Enter final pose in format: x y (matrix indices)\n')
                        final_pose = self.bytes_pose_to_tuple(screen.getstr())
                        
                        # Update poses
                        self.set_current_pose(curr_pose[0], curr_pose[1], curr_pose[2])
                        self.set_goal_pose(final_pose[0], final_pose[1])
                        
                        screen.addstr(f'Updated current pose to ({curr_pose[0]},{curr_pose[1]},{curr_pose[2]})\n')
                        screen.addstr(f'Updated goal pose to ({final_pose[0]},{final_pose[1]})\n')
                        screen.refresh()
                        time.sleep(2)
                    except Exception as e:
                        screen.addstr(f'ERROR: Invalid Input Format!!! {str(e)}\n')
                        screen.refresh()
                        time.sleep(2)
                
        except curses.error as e:
            print(f'Cursor error: {str(e)}')
        finally:
            curses.nocbreak()
            screen.keypad(False)
            curses.echo()
            curses.endwin()

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    
    try:
        # Run the curses interface, which will publish poses when updated
        node.run_curses_interface()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
