import socket
import pickle
import struct
import cv2 as cv
import numpy as np


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

        cv.imshow(imgL)
        cv.imshow(imgR)

    except KeyboardInterrupt:
        break

client_socket.close()
