import numpy as np
import cv2 as cv
import time
import threading
import socket
import pickle
import struct
from picamera2 import Picamera2
import matplotlib.pyplot as plt

# Initialize the socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
laptop_ip = '10.7.26.244'
laptop_port = 8080

# Camera initialization
picam2L = Picamera2(camera_num=0)
picam2R = Picamera2(camera_num=1)

video_config = picam2L.create_video_configuration(main={"size": (640, 480)})
picam2L.configure(video_config)
picam2R.configure(video_config)

picam2L.start()
picam2R.start()

# Connect to laptop
def connect_to_laptop():
    try:
        client_socket.connect((laptop_ip, laptop_port))
        print(f"Connected to laptop at {laptop_ip}:{laptop_port}")
    except Exception as e:
        print(f"Error with socket connect: {e}")

# Send data to laptop
def send_data(imgL, imgR):
    try:
        # Resize images to reduce size for transmission
        imgL_resized = cv.resize(imgL, (320, 240))
        imgR_resized = cv.resize(imgR, (320, 240))

        # Encode the images as JPEG
        _, bufferL = cv.imencode('.jpg', imgL_resized)
        _, bufferR = cv.imencode('.jpg', imgR_resized)

        # Serialize and send the data
        data = {'imgL': bufferL.tobytes(), 'imgR': bufferR.tobytes()}
        serialized_data = pickle.dumps(data)
        data_size = struct.pack('>I', len(serialized_data))
        client_socket.sendall(data_size)
        client_socket.sendall(serialized_data)
    except Exception as e:
        print(f"Error sending data: {e}")

# Capture images in parallel using threading
def capture_images():
    while True:
        imgL = picam2L.capture_array()
        imgR = picam2R.capture_array()
        send_data(imgL, imgR)

# Main loop for disparity calculation
def compute_disparity(imgL, imgR):
    imgL = cv.cvtColor(imgL, cv.COLOR_RGB2BGR)
    imgR = cv.cvtColor(imgR, cv.COLOR_RGB2BGR)
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    # Stereo BM matcher
    stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
    disp = stereo.compute(grayL, grayR)
    disp = np.uint8(disp / 16.0)

    # Display the disparity map
    plt.imshow(disp, 'gray')
    plt.title('Disparity Map')
    plt.show()

# Start camera capture thread and disparity computation loop
def main():
    connect_to_laptop()

    # Create a thread for image capture
    capture_thread = threading.Thread(target=capture_images)
    capture_thread.daemon = True
    capture_thread.start()

    # Main loop for processing and disparity computation
    try:
        while True:
            # Continuously capture images and process them
            imgL = picam2L.capture_array()
            imgR = picam2R.capture_array()
            compute_disparity(imgL, imgR)
            time.sleep(0.05)  # Adjust sleep time to balance processing and real-time feed
    except KeyboardInterrupt:
        pass
    finally:
        picam2L.stop()
        picam2R.stop()
        client_socket.close()

if __name__ == '__main__':
    main()
