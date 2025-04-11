import RPi.GPIO as GPIO
import cv2
import numpy as np
import threading

# Define GPIO pins connected to L298N motor driver
motor_pins = {
    'left': {
        'IN1': 27,
        'IN2': 17,
        'Enable': 22
    },
    'right': {
        'IN1': 24,
        'IN2': 23,
        'Enable': 25
    }
}

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for motor in motor_pins.values():
    GPIO.setup(motor['IN1'], GPIO.OUT)
    GPIO.setup(motor['IN2'], GPIO.OUT)
    GPIO.setup(motor['Enable'], GPIO.OUT)
    motor['PWM'] = GPIO.PWM(motor['Enable'], 1000)

# Start PWM with 0% duty cycle
for motor in motor_pins.values():
    motor['PWM'].start(0)

# Define motor control functions
def move_forward(speed=0.3):
    print("Moving forward")
    for motor in motor_pins.values():
        GPIO.output(motor['IN1'], GPIO.HIGH)
        GPIO.output(motor['IN2'], GPIO.LOW)
        motor['PWM'].ChangeDutyCycle(speed * 100)

def rotate_left(speed=0.3):
    print("Rotating left")
    GPIO.output(motor_pins['left']['IN1'], GPIO.LOW)
    GPIO.output(motor_pins['left']['IN2'], GPIO.HIGH)
    motor_pins['left']['PWM'].ChangeDutyCycle(speed * 100)

def rotate_right(speed=0.3):
    print("Rotating right")
    GPIO.output(motor_pins['right']['IN1'], GPIO.LOW)
    GPIO.output(motor_pins['right']['IN2'], GPIO.HIGH)
    motor_pins['right']['PWM'].ChangeDutyCycle(speed * 100)

def stop_motors():
    print("Stopping motors")
    for motor in motor_pins.values():
        GPIO.output(motor['IN1'], GPIO.LOW)
        GPIO.output(motor['IN2'], GPIO.LOW)
        motor['PWM'].ChangeDutyCycle(0)

# Define global variable for position and lock for thread safety
position = "centre"
position_lock = threading.Lock()
positions = []

def detect_ball_position():
    global position
    print("Executing detect_ball_position thread")
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture video.")
            break

        # Apply Gaussian blur to reduce noise
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        lower_color = np.array([23, 50, 50])
        upper_color = np.array([43, 150, 150])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw double frame
        frame_center_x, frame_center_y = frame.shape[1] // 2, frame.shape[0] // 2
        frame_width, frame_height = frame.shape[1], frame.shape[0]
        cv2.rectangle(frame, (frame_center_x - 50, frame_center_y - 50), (frame_center_x + 50, frame_center_y + 50), (255, 0, 0), 2)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                cv2.putText(frame, "Object detected", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Check if object is within the double frame
                if frame_center_x - 50 < cX < frame_center_x + 50 and frame_center_y - 50 < cY < frame_center_y + 50:
                    position = "stop"
                    print("Object detected within the double frame")
                elif cX < frame.shape[1] // 3:
                    position = "left"
                    print("Ball detected on the left")
                elif cX > 2 * frame.shape[1] // 3:
                    position = "right"
                    print("Ball detected on the right")
                else:
                    position = "centre"
                    print("Ball detected in the centre")

        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

        combined_frame = np.hstack((frame, masked_frame))
        cv2.imshow("Original vs Masked", combined_frame)

        positions.append(position)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def move_robot():
    global position
    print("Executing move_robot thread")
    while True:
        with position_lock:
            ball_position = position
            print("Ball position:", ball_position)
            if ball_position == "left":
                rotate_left()  # Turn left
                move_forward()  # Move forward after turning
            elif ball_position == "centre":
                move_forward()  # Move forward only
            elif ball_position == "right":
                rotate_right()  # Turn right
                move_forward()  # Move forward after turning
            elif ball_position == "stop":
                stop_motors()  # Stop the robot
                  # Exit the loop
            else:
                print("Unknown position")

def main():
    thread1 = threading.Thread(target=detect_ball_position)
    thread2 = threading.Thread(target=move_robot)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        stop_motors()
        GPIO.cleanup()
