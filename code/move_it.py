import RPi.GPIO as GPIO
import time

# --------------------------- Configuration --------------------------- #

GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the servos
LEFT_WHEEL_PIN = 18    
RIGHT_WHEEL_PIN = 17   
# PWM Frequency for servos
PWM_FREQ = 50  
STOP_DUTY = 7.5       
FORWARD_DUTY = 8.5    
BACKWARD_DUTY = 6.5   
TURN_DUTY_OFFSET = 1.0  

DEFAULT_DURATION = 1

# --------------------------- Setup GPIO --------------------------- #

# Set GPIO pins as output
GPIO.setup(LEFT_WHEEL_PIN, GPIO.OUT)
GPIO.setup(RIGHT_WHEEL_PIN, GPIO.OUT)

# Initialize PWM on both servos
left_wheel_pwm = GPIO.PWM(LEFT_WHEEL_PIN, PWM_FREQ)
right_wheel_pwm = GPIO.PWM(RIGHT_WHEEL_PIN, PWM_FREQ)

# Start PWM with neutral position
left_wheel_pwm.start(STOP_DUTY)
right_wheel_pwm.start(STOP_DUTY)
time.sleep(1)

# --------------------------- Movement Functions --------------------------- #

def forward(duration=DEFAULT_DURATION):
    """
    Move the robot forward by setting both servos to move clockwise.
    """
    print("Moving Forward")
    left_wheel_pwm.ChangeDutyCycle(FORWARD_DUTY)   # Clockwise
    right_wheel_pwm.ChangeDutyCycle(FORWARD_DUTY)  # Clockwise
    time.sleep(duration)
    stop()

def backward(duration=DEFAULT_DURATION):
    """
    Move the robot backward by setting both servos to move counter-clockwise.
    """
    print("Moving Backward")
    left_wheel_pwm.ChangeDutyCycle(BACKWARD_DUTY)  # Counter-Clockwise
    right_wheel_pwm.ChangeDutyCycle(BACKWARD_DUTY) # Counter-Clockwise
    time.sleep(duration)
    stop()

def turn_left(duration=DEFAULT_DURATION):
    """
    Turn the robot left by moving the right servo counter-clockwise
    and the left servo clockwise.
    """
    print("Turning Left")
    left_wheel_pwm.ChangeDutyCycle(FORWARD_DUTY)         # Clockwise
    right_wheel_pwm.ChangeDutyCycle(STOP_DUTY - TURN_DUTY_OFFSET)  # Counter-Clockwise
    time.sleep(duration)
    stop()

def turn_right(duration=DEFAULT_DURATION):
    """
    Turn the robot right by moving the left servo counter-clockwise
    and the right servo clockwise.
    """
    print("Turning Right")
    left_wheel_pwm.ChangeDutyCycle(STOP_DUTY - TURN_DUTY_OFFSET)  # Counter-Clockwise
    right_wheel_pwm.ChangeDutyCycle(FORWARD_DUTY)        # Clockwise
    time.sleep(duration)
    stop()

def stop():
    """
    Stop both servos by setting them to the neutral position.
    """
    print("Stopping")
    left_wheel_pwm.ChangeDutyCycle(STOP_DUTY)
    right_wheel_pwm.ChangeDutyCycle(STOP_DUTY)
    time.sleep(0.5)  # Short delay to ensure stop

# --------------------------- Cleanup Function --------------------------- #

def cleanup():
    """
    Stop PWM and clean up GPIO settings.
    """
    left_wheel_pwm.stop()
    right_wheel_pwm.stop()
    GPIO.cleanup()
    print("GPIO Cleanup Completed")

# --------------------------- Main Program --------------------------- #

if __name__ == "__main__":
    try:
        while True:
            print("\nEnter a command:")
            print("f: Forward")
            print("b: Backward")
            print("l: Turn Left")
            print("r: Turn Right")
            print("s: Stop")
            print("q: Quit")
            command = input("Command: ").strip().lower()

            if command == 'f':
                forward()
            elif command == 'b':
                backward()
            elif command == 'l':
                turn_left()
            elif command == 'r':
                turn_right()
            elif command == 's':
                stop()
            elif command == 'q':
                print("Quitting...")
                break
            else:
                print("Invalid command. Please try again.")

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt Detected. Exiting...")

    finally:
        cleanup()
