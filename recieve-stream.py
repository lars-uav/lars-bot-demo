import cv2
import socket
import pickle
import struct
import numpy as np

# Set up the socket to listen for incoming connections
server_ip = '0.0.0.0'  # Listen on all available interfaces
server_port = 5000
server_address = (server_ip, server_port)

# Create the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(server_address)
sock.listen(1)

print("Waiting for connection...")
connection, client_address = sock.accept()

print("Connection established!")

# Receive the video stream
while True:
    # First, receive the size of the frame
    message_size = struct.unpack("L", connection.recv(8))[0]
    
    # Now, receive the frame data
    frame_data = b""
    while len(frame_data) < message_size:
        frame_data += connection.recv(message_size - len(frame_data))

    # Decode the frame
    frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), 1)

    if frame is None:
        print("Error: Could not decode frame")
        break

    # Display the frame
    cv2.imshow("Drone Camera Stream", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
connection.close()
sock.close()
cv2.destroyAllWindows()
