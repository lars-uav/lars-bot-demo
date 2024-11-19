import socket
import serial
import time
import json

class RobotController:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=9600):
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

class RobotServer:
    def __init__(self, host='0.0.0.0', port=5000):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(1)
        self.robot = RobotController()
        print(f"Server listening on {host}:{port}")
        
    def handle_command(self, command):
        command = command.strip().lower()
        if command == 'front':
            return self.robot.front_grid()
        elif command == 'back':
            return self.robot.back_grid()
        elif command == 'right':
            return self.robot.right_grid()
        elif command == 'left':
            return self.robot.left_grid()
        elif command == 'stop':
            return self.robot.emergency_stop()
        else:
            return f"Unknown command: {command}"

    def run(self):
        try:
            while True:
                print("Waiting for connection...")
                client_socket, address = self.server_socket.accept()
                print(f"Connected to {address}")
                
                try:
                    while True:
                        # Receive command
                        data = client_socket.recv(1024).decode()
                        if not data:
                            break
                            
                        # Process command
                        try:
                            command = json.loads(data)['command']
                            print(f"Received command: {command}")
                            
                            # Execute command
                            result = self.handle_command(command)
                            
                            # Send response
                            response = json.dumps({
                                'status': 'success',
                                'message': result
                            })
                            client_socket.send(response.encode())
                            
                        except json.JSONDecodeError:
                            response = json.dumps({
                                'status': 'error',
                                'message': 'Invalid command format'
                            })
                            client_socket.send(response.encode())
                            
                except Exception as e:
                    print(f"Error handling client: {e}")
                finally:
                    client_socket.close()
                    
        except KeyboardInterrupt:
            print("\nShutting down server...")
        finally:
            self.robot.close()
            self.server_socket.close()

if __name__ == "__main__":
    server = RobotServer()
    server.run()