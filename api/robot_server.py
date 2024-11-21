from flask import Flask, request, jsonify
import serial
import time
import traceback
import logging
import sys
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('robot_server.log')
    ]
)

app = Flask(__name__)
app.config['JSON_AS_ASCII'] = False

class RobotController:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=9600):
        self.arduino = None
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.connect()
        
    def connect(self):
        """Establish serial connection with error handling"""
        try:
            # Close existing connection if any
            if self.arduino and self.arduino.is_open:
                self.arduino.close()
                
            # List available serial ports
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            logging.info(f"Available serial ports: {', '.join(str(p) for p in ports)}")
            
            # Attempt to open new connection
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=2)
            time.sleep(2)  # Wait for Arduino connection
            logging.info(f"Connected to Arduino on {self.serial_port}")
            return True
            
        except (serial.SerialException, Exception) as e:
            logging.error(f"Error connecting to Arduino: {e}")
            self.arduino = None
            return False
            
    def is_connected(self):
        """Check if Arduino is connected and port is open"""
        if self.arduino is None or not self.arduino.is_open:
            return False
            
        try:
            # Test if we can write to the port
            self.arduino.write(b'S')  # Send stop command as test
            return True
        except:
            return False

    def _send_command(self, command):
        """Send command with connection retry"""
        if not self.is_connected():
            logging.warning("Arduino not connected, attempting reconnection...")
            if not self.connect():
                return False
                
        try:
            self.arduino.write(command.encode())
            logging.info(f"Sent command: {command}")
            return True
        except Exception as e:
            logging.error(f"Error sending command: {e}")
            return False

    # ... [rest of RobotController methods remain the same] ...

# Global robot controller instance
robot = RobotController()

@app.route('/control', methods=['POST'])
def control_robot():
    """Handle robot control commands"""
    try:
        data = request.json
        command = data.get('command')
        logging.info(f"Received command: {command}")
        
        # Map commands to robot methods
        command_map = {
            'front': robot.front_grid,
            'back': robot.back_grid,
            'right': robot.right_grid,
            'left': robot.left_grid,
            'stop': robot.emergency_stop
        }
        
        if command == 'check_connection':
            connected = robot.is_connected()
            logging.info(f"Connection check: {connected}")
            return jsonify({
                'status': 'success',
                'connected': connected
            }), 200
            
        if command in command_map:
            result = command_map[command]()
            logging.info(f"Command result: {result}")
            return jsonify({
                'status': 'success',
                'message': result
            }), 200
        else:
            logging.warning(f"Unknown command received: {command}")
            return jsonify({
                'status': 'error',
                'message': f'Unknown command: {command}'
            }), 400
            
    except Exception as e:
        logging.error(f"Error in control_robot: {traceback.format_exc()}")
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

@app.route('/', methods=['GET'])
def health_check():
    """Health check endpoint"""
    connected = robot.is_connected()
    logging.info(f"Health check - Arduino connected: {connected}")
    return jsonify({
        'status': 'ok',
        'connected': connected
    }), 200

if __name__ == '__main__':
    # Get port from environment variable or use default
    port = int(os.environ.get('ROBOT_SERVER_PORT', 8001))
    host = os.environ.get('ROBOT_SERVER_HOST', '0.0.0.0')
    
    logging.info(f"Starting robot server on {host}:{port}")
    
    try:
        app.run(host=host, port=port, debug=True)
    except Exception as e:
        logging.error(f"Server failed to start: {e}")
    finally:
        robot.close()