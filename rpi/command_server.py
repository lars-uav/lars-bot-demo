import socket
import serial
import time
import json
import threading
import struct

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

def send_response(client_socket, response_dict):
    """Send a response using the same protocol structure as the video server"""
    try:
        # Convert response to JSON and encode
        response_data = json.dumps(response_dict).encode()
        
        # Pack and send the message size first
        message_size = struct.pack("L", len(response_data))
        client_socket.sendall(message_size + response_data)
        return True
    except (ConnectionResetError, BrokenPipeError, socket.error):
        return False

def handle_client(client_socket, addr, robot):
    """Handle individual client connection"""
    print(f'Handling connection from {addr}')
    
    try:
        while True:
            # Receive message size first (long integer)
            size_data = client_socket.recv(struct.calcsize("L"))
            if not size_data:
                break
                
            # Unpack message size
            message_size = struct.unpack("L", size_data)[0]
            
            # Receive the actual command data
            received_data = b""
            while len(received_data) < message_size:
                chunk = client_socket.recv(min(message_size - len(received_data), 4096))
                if not chunk:
                    raise ConnectionError("Connection broken")
                received_data += chunk
            
            # Process command
            try:
                command = json.loads(received_data.decode())['command']
                print(f"Received command: {command}")
                
                # Execute command
                result = ""
                if command == 'front':
                    result = robot.front_grid()
                elif command == 'back':
                    result = robot.back_grid()
                elif command == 'right':
                    result = robot.right_grid()
                elif command == 'left':
                    result = robot.left_grid()
                elif command == 'stop':
                    result = robot.emergency_stop()
                else:
                    result = f"Unknown command: {command}"
                
                # Send response
                response = {
                    'status': 'success',
                    'message': result
                }
                if not send_response(client_socket, response):
                    break
                
            except json.JSONDecodeError:
                response = {
                    'status': 'error',
                    'message': 'Invalid command format'
                }
                if not send_response(client_socket, response):
                    break
                    
    except Exception as e:
        print(f"Error with client {addr}: {e}")
    finally:
        client_socket.close()
        print(f"Connection closed for {addr}")

def start_robot_server(host='0.0.0.0', port=5000):
    print("Initializing robot controller...")
    robot = RobotController()
    print("Robot controller initialized successfully!")
    
    # Create socket server
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((host, port))
        server_socket.listen(5)
        print(f"Server listening on {host}:{port}")
    except Exception as e:
        print(f"Socket error: {e}")
        robot.close()
        return
        
    # Accept multiple clients
    try:
        while True:
            print("Waiting for a new client connection...")
            client_socket, addr = server_socket.accept()
            print(f"Accepted connection from {addr}")
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, addr, robot)
            )
            client_thread.daemon = True
            client_thread.start()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        robot.close()
        server_socket.close()

if __name__ == "__main__":
    start_robot_server()
