import RPi.GPIO as GPIO
from os.path import isfile, join
import numpy as np
import cv2 as cv
from cv2 import ximgproc
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from picamera2 import Picamera2
import time

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
yfloor = 340

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
RL = np.loadtxt(join(PATH_CALIB, 'RectifL.txt'), dtype=np.float32)
CL = np.loadtxt(join(PATH_CALIB, 'CmL.txt'), dtype=np.float32)
DL = np.loadtxt(join(PATH_CALIB, 'DcL.txt'), dtype=np.float32)

''' Servo Control Setup '''
SERVO_PIN = 18  # Change to your servo GPIO pin
SERVO_FREQ = 50  # Servo frequency (Hz)
CENTER_PULSE = 7.5  # Neutral position (adjust based on servo)
LEFT_PULSE = 10.0   # Leftmost position
RIGHT_PULSE = 5.0   # Rightmost position

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
servo.start(CENTER_PULSE)  # Start at the neutral position


''' End Global Variables '''

def main():
    # Initialize the cameras
    picam2L = Picamera2(camera_num=0)
    picam2R = Picamera2(camera_num=1)

    video_config = picam2L.create_video_configuration(main={"size": (640, 480)})
    picam2L.configure(video_config)
    picam2R.configure(video_config)

    picam2L.start()
    picam2R.start()

    plt.figure(figsize=(16, 9))

    try:
        while True:
            # Capture frames from both cameras
            imgL = picam2L.capture_array()
            imgR = picam2R.capture_array()

            # Process the frames and calculate commands
            steering_command = compute_disparity(imgL, imgR, params)

            # Send servo command
            control_servo(steering_command)

            # Update the plots
            plt.pause(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        picam2L.stop()
        picam2R.stop()
        servo.stop()
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
    dispL = stereoL.compute(grayL, grayR)

     ### Init WLS Filter with parameters
    wls = ximgproc.createDisparityWLSFilter(stereoL)
    stereoR = ximgproc.createRightMatcher(stereoL)
    wls.setLambda(lam)
    wls.setDepthDiscontinuityRadius(discontinuityRad)  # Default 4
    wls.setSigmaColor(sigma)

    ### Compute raw disparity from both sides
    ts1 = time.time()
    dispL = stereoL.compute(grayL, grayR)
    dispR = stereoR.compute(grayR, grayL)
    ts2 = time.time()
    cost_sgbm = ts2 - ts1

    ### Filter raw disparity using weighted least squares based smoothing
    dispFinal = wls.filter(dispL, imgL, None, dispR)
    dispFinal = ximgproc.getDisparityVis(dispFinal)

    paramsVals = [sigma, lam,
                  stereoL.getNumDisparities(), stereoL.getBlockSize(),
                  stereoL.getPreFilterCap(), stereoL.getSpeckleRange()]

    ''' Map disparity values to depth as 3D point cloud '''
    points3d = cv.reprojectImageTo3D(
        dispFinal, Q, ddepth=cv.CV_32F, handleMissingValues=True)
    
    return find_path(points3d)


def rescaleROI(src, roi):
    x, y, w, h = roi
    return src[y:y+h, x:x+w]


def find_path(points3d):
    xx, yy, zz = points3d[:, :, 0], points3d[:, :, 1], points3d[:, :, 2]
    xx, yy, zz = np.clip(xx, -25, 60), np.clip(yy, -25, 25), np.clip(zz, 0, 100)

    # Compute the straight-line direction
    center_index = zz.shape[1] // 2
    left_obstacle = np.min(zz[:, :center_index])
    right_obstacle = np.min(zz[:, center_index:])

    if left_obstacle > right_obstacle:
        return LEFT_PULSE  # Turn left
    elif right_obstacle > left_obstacle:
        return RIGHT_PULSE  # Turn right
    else:
        return CENTER_PULSE  # Move straight


def control_servo(command):
    servo.ChangeDutyCycle(command)
    print(f"Servo set to pulse: {command}")
    time.sleep(0.5)  # Small delay for the servo to move
