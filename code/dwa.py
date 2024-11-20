import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# GPIO Pin Setup for MG996R Servos
LEFT_MOTOR_PWM_PIN = 18  # GPIO pin for left motor PWM
RIGHT_MOTOR_PWM_PIN = 17  # GPIO pin for right motor PWM
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR_PWM_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PWM_PIN, GPIO.OUT)

# Initialize the GPIO PWM for servos (50 Hz for MG996R)
left_pwm = GPIO.PWM(LEFT_MOTOR_PWM_PIN, 50)  # 50 Hz
right_pwm = GPIO.PWM(RIGHT_MOTOR_PWM_PIN, 50)
left_pwm.start(7.5)  # Neutral position (stop)
right_pwm.start(7.5)  # Neutral position (stop)

# Initialize cameras
camera1 = Picamera2(0)  # First camera
camera2 = Picamera2(1)  # Second camera
camera1.start()
camera2.start()

# Create StereoBM object for disparity computation
stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)

# Constants
WHEEL_RADIUS = 0.035 # in meters (example, adjust according to your wheels)
WHEELBASE = 0.22  # distance between wheels in meters
PWM_TO_SPEED_CONSTANT = 0.1  # adjust according to your PWM to speed mapping

# Function to set servo control
def set_servo_control(left_pwm_value, right_pwm_value):
    left_pwm.ChangeDutyCycle(left_pwm_value)
    right_pwm.ChangeDutyCycle(right_pwm_value)
    print(f"Left Motor PWM: {left_pwm_value}, Right Motor PWM: {right_pwm_value}")

# Function to calculate velocities
def calculate_velocities(left_pwm, right_pwm):
    V_L = PWM_TO_SPEED_CONSTANT * left_pwm  # Linear velocity of left wheel
    V_R = PWM_TO_SPEED_CONSTANT * right_pwm  # Linear velocity of right wheel
    V = (V_L + V_R) / 2  # Average linear velocity
    omega = (V_R - V_L) / WHEELBASE  # Angular velocity
    return V, omega

def follow_the_gap(depth_map):
    height, width = depth_map.shape
    depth_row = np.mean(depth_map, axis=0)  # Take the average depth along the horizontal axis
    
    # Step 1: Find the closest obstacle
    nearest_obstacle_idx = np.argmin(depth_row)
    
    # Step 2: Set all points within the safety bubble to zero
    safety_radius = 10  # Define safety radius (in pixels)
    depth_row[max(0, nearest_obstacle_idx - safety_radius):min(width, nearest_obstacle_idx + safety_radius)] = 0
    
    # Step 3: Find the largest gap
    free_space = (depth_row > 0).astype(int)  # Binary map of free space
    free_space_diff = np.diff(np.concatenate(([0], free_space, [0])))  # Find gap start and end points
    gap_starts = np.where(free_space_diff == 1)[0]
    gap_ends = np.where(free_space_diff == -1)[0]
    
    # Calculate the length of each gap and find the largest
    if len(gap_starts) > 0 and len(gap_ends) > 0:
        largest_gap_idx = np.argmax(gap_ends - gap_starts)
        largest_gap_start = gap_starts[largest_gap_idx]
        largest_gap_end = gap_ends[largest_gap_idx]
        gap_center = (largest_gap_start + largest_gap_end) // 2
        
        # Debugging: Print the gap information
        print(f"Largest Gap Start: {largest_gap_start}, End: {largest_gap_end}, Center: {gap_center}")
        
        # Step 4: Command the bot to head towards the center of the largest gap
        if gap_center < width // 2- 30:
            # Turn left
            set_servo_control(6.5, 8.5)  # Adjust based on your motor behavior
        elif gap_center > width // 2 +30:
            # Turn right
            set_servo_control(8.5, 6.5)
        else:
            # Go straight
            set_servo_control(7.5, 7.5)
    else:
        # No valid gap found, stop the robot
        set_servo_control(7.5, 7.5)
        print("No gap found, stopping the bot")

try:
    while True:
        # Capture frames from both cameras
        frame1 = camera1.capture_array()
        frame2 = camera2.capture_array()

        # Convert to grayscale
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Compute the disparity map
        disparity = stereo.compute(gray1, gray2)

        # Normalize the disparity map for visualization
        disparity = cv2.normalize(disparity, disparity, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX)
        disparity = np.uint8(disparity)

        # Estimate depth from disparity (assume focal_length and baseline)
        focal_length = 3389  # rpi camera module 3
        baseline = 0.1  # Example baseline in meters
        depth_map = (focal_length * baseline) / (disparity + 1e-6)  # Avoid division by zero

        # Visualize disparity and depth map
        cv2.imshow("Disparity Map", disparity)

        # Apply the Follow-the-Gap method
        follow_the_gap(depth_map)

        # Calculate velocities based on motor commands
        V, omega = calculate_velocities(7.5, 7.5)  # Replace with actual PWM values
        print(f"Linear Velocity: {V}, Angular Velocity: {omega}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
    camera1.close()
    camera2.close()
    cv2.destroyAllWindows()
