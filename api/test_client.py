import requests
import time
import logging

logging.basicConfig(level=logging.INFO)

class RobotClient:
    def __init__(self, host, port=8001):
        self.base_url = f"http://{host}:{port}"
        self.session = requests.Session()
        
    def check_connection(self):
        """Check if server is reachable and robot is connected"""
        try:
            response = self.session.get(f"{self.base_url}/")
            data = response.json()
            return data.get('connected', False)
        except Exception as e:
            logging.error(f"Connection check failed: {e}")
            return False
            
    def send_command(self, command):
        """Send command to robot"""
        try:
            response = self.session.post(
                f"{self.base_url}/command",
                json={'command': command},
                timeout=5
            )
            response.raise_for_status()
            result = response.json()
            logging.info(f"Command result: {result}")
            return result['status'] == 'success'
        except Exception as e:
            logging.error(f"Command failed: {e}")
            return False

def main():
    # Initialize robot client
    robot_ip = input("Enter robot IP address: ")  # e.g., "10.1.59.194"
    robot = RobotClient(robot_ip)
    
    # Check connection
    if not robot.check_connection():
        logging.error("Failed to connect to robot")
        return
        
    print("\nRobot Control:")
    print("Commands: F (Forward), B (Back), L (Left), R (Right), S (Stop), Q (Quit)")
    
    while True:
        command = input("\nEnter command: ").upper()
        
        if command == 'Q':
            break
            
        if command in ['F', 'B', 'L', 'R', 'S']:
            if robot.send_command(command):
                print("Command executed successfully")
            else:
                print("Command failed")
        else:
            print("Invalid command")

if __name__ == "__main__":
    main()