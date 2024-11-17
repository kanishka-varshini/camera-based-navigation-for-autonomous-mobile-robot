import socket
import pickle
import struct
import cv2 as cv
import numpy as np
import sys

sys.path.append('/path/to/ORB_SLAM2/Python')

import orbslam2  # ORB-SLAM2 Python bindings

def receive_data(client_socket):
    try:
        # First, receive the size of the data
        data_size = struct.unpack('>I', client_socket.recv(4))[0]
        
        # Receive the actual serialized data based on the size
        serialized_data = b""
        while len(serialized_data) < data_size:
            remaining_data = client_socket.recv(data_size - len(serialized_data))
            if not remaining_data:
                break
            serialized_data += remaining_data
        
        # Deserialize the received data
        data = pickle.loads(serialized_data)
        imgL_bytes = data['imgL']
        imgR_bytes = data['imgR']

        # Decode the images from the byte data
        imgL = cv.imdecode(np.frombuffer(imgL_bytes, np.uint8), cv.IMREAD_COLOR)
        imgR = cv.imdecode(np.frombuffer(imgR_bytes, np.uint8), cv.IMREAD_COLOR)

        return imgL, imgR

    except Exception as e:
        print(f"An error occurred while receiving data: {e}")
        return None, None



# Initialize ORB-SLAM2
vocab_file = '/path/to/ORB_SLAM2/Vocabulary/ORBvoc.bin'
settings_file = '/path/to/ORB_SLAM2/config.yaml'
slam_system = orbslam2.System(vocab_file, settings_file, orbslam2.System.STEREO)

# Start ORB-SLAM2 system
slam_system.set_use_viewer(True)
slam_system.initialize()

# Set up the socket to receive images
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.bind(('0.0.0.0', 8080))
client_socket.listen(1)
print("Waiting for connection...")
client_socket, addr = client_socket.accept()
print(f"Connection established with {addr}")

while True:
    try:
       
        imgL, imgR=receive_data(client_socket)

        # Process the images with ORB-SLAM2
        slam_system.process_image_left(imgL)
        slam_system.process_image_right(imgR)

        # Optionally visualize the map or pose
        slam_system.get_trajectory()

    except KeyboardInterrupt:
        break

# Close the connection and stop the SLAM system
client_socket.close()
slam_system.shutdown()
