import cv2
import numpy as np
import RPi.GPIO as GPIO
from MotorModule import Motor 

motor= Motor(2,3,4,17,22,27)   

# -------------------------
# Motor GPIO Pins
# -------------------------


# -------------------------
# Helper functions
# -------------------------
def stop_motors():
    motor.move(0,0)
   
def move_left():
    motor.move(0.5, 0.3)
    
def move_straight():
   motor.move(0.5,0)

def move_right():
    motor.move(0.5, -0.3)
    

# -------------------------
# Camera Setup - Using picamera2 with OpenCV fallback
# -------------------------
# Try picamera2 first (Raspberry Pi)
picam2 = None
cap = None
use_picamera2 = False
frame_width = 160
frame_height = 120

try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (frame_width, frame_height), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    use_picamera2 = True
    print(f"[CAMERA] Using picamera2 ({frame_width}x{frame_height})")
except ImportError:
    print("[WARN] picamera2 not available. Trying OpenCV VideoCapture...")
    picam2 = None
except Exception as exc:
    print(f"[WARN] picamera2 initialization failed: {exc}")
    print("[WARN] Falling back to OpenCV VideoCapture...")
    if picam2:
        try:
            picam2.close()
        except Exception:
            pass
        picam2 = None

# Fallback to OpenCV VideoCapture if picamera2 failed
if not use_picamera2:
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        print(f"[CAMERA] Using OpenCV VideoCapture")
    else:
        print("Error: Cannot open camera")
        GPIO.cleanup()
        exit()

def read_frame():
    """
    Read a frame from camera (picamera2 or OpenCV).
    Returns (success, frame) where frame is BGR format.
    """
    if use_picamera2 and picam2:
        try:
            # picamera2 returns RGB888 format
            rgb_array = picam2.capture_array()
            # Convert RGB to BGR for OpenCV compatibility
            bgr_frame = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            return True, bgr_frame
        except Exception as exc:
            print(f"[WARN] picamera2 capture failed: {exc}")
            return False, None
    elif cap:
        return cap.read()
    return False, None

# -------------------------
# Main Loop
# -------------------------
try:
    while True:
        ret, frame = read_frame()
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
    # Cleanup camera
    if use_picamera2 and picam2:
        try:
            picam2.stop()
            picam2.close()
        except Exception:
            pass
    elif cap:
        cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()