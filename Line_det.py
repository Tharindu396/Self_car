import cv2
import numpy as np
import RPi.GPIO as GPIO

# -------------------------
# Motor GPIO Pins
# -------------------------
in1 = 4
in2 = 17
in3 = 27
in4 = 22
en1 = 23
en2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

p1 = GPIO.PWM(en1, 100)
p2 = GPIO.PWM(en2, 100)
p1.start(50)
p2.start(50)

# -------------------------
# Helper functions
# -------------------------
def stop_motors():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

def move_left():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def move_straight():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def move_right():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

# -------------------------
# Camera Setup
# -------------------------
cap = cv2.VideoCapture("libcamerasrc ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

cap.set(3, 160)  # width
cap.set(4, 120)  # height

if not cap.isOpened():
    print("Error: Cannot open camera")
    GPIO.cleanup()
    exit()

# -------------------------
# Main Loop
# -------------------------
try:
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to grab frame")
            continue

        # -------------------------
        # Line detection
        # -------------------------
        # Adjust these values to match your line color
        #low_b = np.array([0,0,0], dtype=np.uint8)
        #high_b = np.array([5,5,5], dtype=np.uint8)
        #mask = cv2.inRange(frame, low_b, high_b)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_hsv = np.array([0,0,0])
        high_hsv = np.array([180,255,50])
        mask = cv2.inRange(hsv, low_hsv, high_hsv)


        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print(f"CX: {cx}  CY: {cy}")

                # -------------------------
                # Motor control logic
                # -------------------------
                if cx >= 120:
                    print("Turn Left")
                    move_left()
                elif 40 < cx < 120:
                    print("On Track!")
                    move_straight()
                elif cx <= 40:
                    print("Turn Right")
                    move_right()

                cv2.circle(frame, (cx, cy), 5, (255,255,255), -1)
                cv2.drawContours(frame, [c], -1, (0,255,0), 1)
        else:
            print("Line not detected")
            stop_motors()

        cv2.imshow("Mask", mask)
        cv2.imshow("Frame", frame)

        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_motors()
            break

except KeyboardInterrupt:
    print("Interrupted!")

finally:
    stop_motors()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()