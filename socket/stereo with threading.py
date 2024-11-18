import threading
import time
import numpy as np
import cv2 as cv
import pickle
import socket
import struct
from picamera2 import Picamera2

# Socket setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
laptop_ip = '10.7.26.244'
laptop_port = 8080

# Camera setup
picam2L = Picamera2(camera_num=0)
picam2R = Picamera2(camera_num=1)
video_config = picam2L.create_video_configuration(main={"size": (640, 480)})
picam2L.configure(video_config)
picam2R.configure(video_config)
picam2L.start()
picam2R.start()

# Connect to laptop
client_socket.connect((laptop_ip, laptop_port))

# Global variables
running = True

def capture_images():
    global running
    while running:
        # Capture frames from both cameras
        imgL = picam2L.capture_array()
        imgR = picam2R.capture_array()

        # Send images to laptop for SLAM processing
        send_data(imgL, imgR)

        # Wait for the next frame
        time.sleep(0.1)

def send_data(imgL, imgR):
    try:
        # Encode images as JPEG to reduce size
        _, bufferL = cv.imencode('.jpg', imgL)
        _, bufferR = cv.imencode('.jpg', imgR)

        # Serialize images and send
        data = {'imgL': bufferL.tobytes(), 'imgR': bufferR.tobytes()}
        serialized_data = pickle.dumps(data)
        data_size = struct.pack('>I', len(serialized_data))

        # Send data size and then the serialized images
        client_socket.sendall(data_size)
        client_socket.sendall(serialized_data)

    except Exception as e:
        print(f"Error in send_data: {e}")

def process_disparity(imgL, imgR):
    # Preprocess images (undistort, rectification, etc.)
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    # Stereo Matching
    stereo = cv.StereoBM_create(numDisparities=50, blockSize=9)
    dispL = stereo.compute(grayL, grayR)

    # Apply WLS filter for disparity refinement
    # Create a disparity object for the right image
    stereo_right = cv.StereoBM_create(numDisparities=50, blockSize=9)
    dispR = stereo_right.compute(grayR, grayL)

    # Create the WLS filter
    wls_filter = cv.ximgproc.createDisparityWLSFilter(stereo)

    # Apply the WLS filter
    filtered_disp = wls_filter.filter(dispL, grayL, disparity_map_right=dispR)

    # Normalize and display the filtered disparity map
    dispFinal = cv.normalize(filtered_disp, None, 0, 255, cv.NORM_MINMAX)
    dispFinal = np.uint8(dispFinal)

    # Display disparity map
    cv.imshow('Disparity Map with WLS', dispFinal)

def main():
    # Start image capture in a separate thread
    capture_thread = threading.Thread(target=capture_images)
    capture_thread.start()

    try:
        while True:
            # Capture images from both cameras
            imgL = picam2L.capture_array()
            imgR = picam2R.capture_array()

            # Display the live feed
            cv.imshow('Left Camera Feed', imgL)
            cv.imshow('Right Camera Feed', imgR)

            # Process disparity map with WLS filter
            process_disparity(imgL, imgR)

            # Wait for a key press and handle quitting
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        # Stop the threads gracefully
        global running
        running = False
        capture_thread.join()

    finally:
        picam2L.stop()
        picam2R.stop()
        client_socket.close()
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()
