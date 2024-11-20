import RPi.GPIO as GPIO
from os.path import join
import numpy as np
import cv2 as cv
from cv2 import ximgproc
import time
from picamera2 import Picamera2

''' Global Variables '''
PATH_CALIB = r'/home/sg/project/Calibration_Files_expm'

### Stereo Matcher Parameters
minDisp = 0
nDisp = 16
bSize = 7
P1 = 8 * 3 * bSize ** 2
P2 = 32 * 3 * bSize ** 2
modeSgbm = cv.StereoSGBM_MODE_SGBM
pfCap = 0
sRange = 0
lam = 32000
sigma = 2.5
discontinuityRad = 4
params = [minDisp, nDisp, bSize, pfCap, sRange]

### Load Camera Calibration Parameters
undistL = np.loadtxt(join(PATH_CALIB, 'umapL.txt'), dtype=np.float32)
rectifL = np.loadtxt(join(PATH_CALIB, 'rmapL.txt'), dtype=np.float32)
undistR = np.loadtxt(join(PATH_CALIB, 'umapR.txt'), dtype=np.float32)
rectifR = np.loadtxt(join(PATH_CALIB, 'rmapR.txt'), dtype=np.float32)
roiL = np.loadtxt(join(PATH_CALIB, 'ROIL.txt'), dtype=int)
roiR = np.loadtxt(join(PATH_CALIB, 'ROIR.txt'), dtype=int)
Q = np.loadtxt(join(PATH_CALIB, 'Q.txt'), dtype=np.float32)

''' Servo Control Setup '''
LEFT_SERVO_PIN = 18  # GPIO pin for left servo
RIGHT_SERVO_PIN = 19  # GPIO pin for right servo
SERVO_FREQ = 50  # Servo frequency (Hz)

# Define duty cycles for different motions
FORWARD_PULSE = 7.5  # Neutral position (move forward)
LEFT_TURN_LEFT_WHEEL = 6.5  # Left wheel backward for left turn
LEFT_TURN_RIGHT_WHEEL = 8.5  # Right wheel forward for left turn
RIGHT_TURN_LEFT_WHEEL = 8.5  # Left wheel forward for right turn
RIGHT_TURN_RIGHT_WHEEL = 6.5  # Right wheel backward for right turn
STOP_PULSE = 7.5  # Neutral for stopping

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_SERVO_PIN, GPIO.OUT)
GPIO.setup(RIGHT_SERVO_PIN, GPIO.OUT)

left_servo = GPIO.PWM(LEFT_SERVO_PIN, SERVO_FREQ)
right_servo = GPIO.PWM(RIGHT_SERVO_PIN, SERVO_FREQ)
left_servo.start(STOP_PULSE)
right_servo.start(STOP_PULSE)

''' Main Program '''
def main():
    # Initialize the cameras
    picam2L = Picamera2(camera_num=0)
    picam2R = Picamera2(camera_num=1)

    video_config = picam2L.create_video_configuration(main={"size": (640, 480)})
    picam2L.configure(video_config)
    picam2R.configure(video_config)

    picam2L.start()
    picam2R.start()

    try:
        while True:
            # Capture frames from both cameras
            imgL = picam2L.capture_array()
            imgR = picam2R.capture_array()

            # Process the frames and calculate commands
            steering_command = compute_disparity(imgL, imgR, params)

            # Send servo command
            control_servos(steering_command)

    except KeyboardInterrupt:
        pass
    finally:
        picam2L.stop()
        picam2R.stop()
        left_servo.stop()
        right_servo.stop()
        GPIO.cleanup()
        print("Exiting program and cleaning up GPIO.")


def compute_disparity(imgL, imgR, params):
    imgL = cv.cvtColor(imgL, cv.COLOR_RGB2BGR)
    imgR = cv.cvtColor(imgR, cv.COLOR_RGB2BGR)

    imgL = cv.remap(imgL, undistL, rectifL, cv.INTER_LINEAR)
    imgR = cv.remap(imgR, undistR, rectifR, cv.INTER_LINEAR)

    imgL = rescaleROI(imgL, roiL)
    imgR = rescaleROI(imgR, roiR)

    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    stereoL = cv.StereoBM_create(numDisparities=nDisp, blockSize=bSize)
    stereoR = ximgproc.createRightMatcher(stereoL)
    wls = ximgproc.createDisparityWLSFilter(stereoL)

    dispL = stereoL.compute(grayL, grayR)
    dispR = stereoR.compute(grayR, grayL)
    dispFiltered = wls.filter(dispL, imgL, None, dispR)

    points3d = cv.reprojectImageTo3D(dispFiltered, Q)
    return find_path(points3d)


def rescaleROI(src, roi):
    x, y, w, h = roi
    return src[y:y+h, x:x+w]


def find_path(points3d):
    zz = np.clip(points3d[:, :, 2], 0, 100)
    center_index = zz.shape[1] // 2
    left_obstacle = np.min(zz[:, :center_index])
    right_obstacle = np.min(zz[:, center_index:])

    if left_obstacle > right_obstacle:
        return "LEFT"
    elif right_obstacle > left_obstacle:
        return "RIGHT"
    else:
        return "FORWARD"


def control_servos(command):
    if command == "LEFT":
        left_servo.ChangeDutyCycle(LEFT_TURN_LEFT_WHEEL)
        right_servo.ChangeDutyCycle(LEFT_TURN_RIGHT_WHEEL)
    elif command == "RIGHT":
        left_servo.ChangeDutyCycle(RIGHT_TURN_LEFT_WHEEL)
        right_servo.ChangeDutyCycle(RIGHT_TURN_RIGHT_WHEEL)
    else:  # FORWARD
        left_servo.ChangeDutyCycle(FORWARD_PULSE)
        right_servo.ChangeDutyCycle(FORWARD_PULSE)
    print(f"Command: {command}")
    time.sleep(0.5)  # Small delay for stability

if __name__ == "__main__":
    main()
