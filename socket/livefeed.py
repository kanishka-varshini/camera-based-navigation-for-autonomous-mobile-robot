from os.path import isfile, join
import numpy as np
import cv2 as cv
from cv2 import ximgproc
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import time
from picamera2 import Picamera2

# setting up communication between pi and laptop for SLAM
import socket
import pickle
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
laptop_ip = '10.7.30.173'  # laptop's IP address 
laptop_port = 8080  #change




def main():
    # Initialize the cameras
    picam2L = Picamera2(camera_num=0)
    picam2R = Picamera2(camera_num=1)

    # Configure the cameras with the same settings
    video_config = picam2L.create_video_configuration(main={"size": (640, 480)})
    picam2L.configure(video_config)
    picam2R.configure(video_config)

    # Start the cameras
    picam2L.start()
    picam2R.start()

    plt.figure(figsize=(16,9))

    #initialize communications between Rpi and laptop
    try:
        client_socket.connect((laptop_ip, laptop_port))
        print(f"Connected to the laptop server at {laptop_ip}:{laptop_port}")

    except Exception as e:
        print(f"An error occurred with socket connect: {e}")

    try:
        while True:
            # Capture frames from both cameras
            imgL = picam2L.capture_array()
            imgR = picam2R.capture_array()

            # sending the raw images to laptop
            send_data(imgL,imgR)

            # Update the plots
            plt.pause(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the cameras
        picam2L.stop()
        picam2R.stop()
        #stop the Rpi-laptop connection
        client_socket.close()
        print("Connection closed.")


def send_data(imgL,imgR):
    
    try:
        # sending data to laptop

        # Encode the images as JPEG to reduce the size
        _, bufferL = cv.imencode('.jpg', imgL)
        _, bufferR = cv.imencode('.jpg', imgR)

        # Serialize the images with pickle
        data = {'imgL': bufferL.tobytes(), 'imgR': bufferR.tobytes()}
        serialized_data = pickle.dumps(data)

        # Send the size of the serialized data first (4 bytes for size)
        data_size = struct.pack('>I', len(serialized_data))
        client_socket.sendall(data_size)
        
        # Now send the actual serialized data
        client_socket.sendall(serialized_data)
    
        # receive response from the laptop  #CHANGE THIS IF GOAL POSE PROVIDING ETC
        # response = client_socket.recv(1024).decode('utf-8')
        # print(f"Received from Laptop: {response}")

    except Exception as e:
        print(f"An error occurred while sending data to laptop: {e}")


main()