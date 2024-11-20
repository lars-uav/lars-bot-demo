from flask import Flask, request, jsonify
import serial
import time

app = Flask(__name__)

class RobotController:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=9600):
        self.arduino = serial.Serial(serial_port, baud_rate)
        time.sleep(2)  # Wait for Arduino connection to establish
        
    def front_grid(self):
        self.arduino.write(b'F')
        time.sleep(3.38)
        self.arduino.write(b'S')
        return "front_grid completed"

    def back_grid(self):
        self.arduino.write(b'B')
        time.sleep(3.36)
        self.arduino.write(b'S')
        return "back_grid completed"

    def right_grid(self):
        self.arduino.write(b'R')
        time.sleep(2.15)
        self.arduino.write(b'S')
        time.sleep(0.2)
        self.arduino.write(b'F')
        time.sleep(0.2)
        self.arduino.write(b'S')
        return "right_grid completed"
        
    def left_grid(self):
        self.arduino.write(b'L')
        time.sleep(2)
        self.arduino.write(b'S')
        return "left_grid completed"
        
    def emergency_stop(self):
        self.arduino.write(b'S')
        return "emergency_stop completed"
        
    def close(self):
        self.arduino.close()

# Global robot controller instance
robot = RobotController()

@app.route('/control', methods=['POST'])
def control_robot():
    """Handle robot control commands"""
    try:
        data = request.json
        command = data.get('command')
        
        # Map commands to robot methods
        command_map = {
            'front': robot.front_grid,
            'back': robot.back_grid,
            'right': robot.right_grid,
            'left': robot.left_grid,
            'stop': robot.emergency_stop
        }
        
        # Execute command
        if command in command_map:
            result = command_map[command]()
            return jsonify({
                'status': 'success', 
                'message': result
            }), 200
        else:
            return jsonify({
                'status': 'error', 
                'message': f'Unknown command: {command}'
            }), 400
    
    except Exception as e:
        return jsonify({
            'status': 'error', 
            'message': str(e)
        }), 500

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        robot.close()