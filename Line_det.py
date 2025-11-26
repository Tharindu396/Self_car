import cv2
import time
import RPi.GPIO as GPIO
from MotorModule import Motor 
from CameraWrapper import CameraWrapper
from typing import Optional
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    cv2 = None
    np = None

motor= Motor(2,3,4,17,22,27)   

class LineFollower:
    """
    Contour-based line follower reusing the logic from Line_det.py

    - Detects the darkest line on the floor using HSV thresholding
    - Adjusts the motor turn value to stay centered on the line
    - Optionally provides frames for AprilTag detection
    """

    def __init__(
        self,
        camera_source: Optional[str] = None,  # Deprecated, kept for compatibility
        frame_width: int = 160,
        frame_height: int = 120,
        base_speed: float = 0.45,
        turn_gain: float = 0.65,
        max_turn: float = 0.5,
        show_display: bool = True,
    ):
        if not CV2_AVAILABLE or np is None:
            raise RuntimeError("OpenCV + NumPy are required for physical navigation mode.")

        self.base_speed = base_speed
        self.turn_gain = turn_gain
        self.max_turn = max_turn
        self.low_hsv = np.array([0,0,0])
        self.high_hsv = np.array([180,255,90])
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.last_detection_time = time.time()
        self.lost_timeout = 0.75
        self.last_frame = None  # Store last frame for AprilTag detection
        self.show_display = show_display
        self.last_mask = None  # Store mask for display
        self.motor = Motor(2, 3, 4, 27, 17, 22)

        # Use CameraWrapper for picamera2/OpenCV compatibility
        try:
            self.camera = CameraWrapper(frame_width, frame_height)
            self.is_active = self.camera.is_initialized
        except RuntimeError as exc:
            self.camera = None
            self.is_active = False
            raise RuntimeError(f"Unable to initialize camera: {exc}") from exc

    def get_last_frame(self):
        """Get the last captured frame for AprilTag detection."""
        return self.last_frame

    @property
    def is_ready(self) -> bool:
        return bool(self.camera and self.camera.is_opened())

    def step(self) -> bool:
        """
        Process one camera frame and adjust motors.

        Returns:
            True if the line is currently detected, False otherwise.
        """
        if not self.is_ready:
            return False

        ret, frame = self.camera.read()
        if not ret or frame is None:
            print("[WARN] Failed to grab frame from camera.")
            self.motor.stop()
            self.last_frame = None
            self.last_mask = None
            return False

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low_hsv, self.high_hsv)
        self.last_mask = mask.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                error = (cx - (self.frame_width / 2)) / max(self.frame_width / 2, 1)
                turn = -error * self.turn_gain
                turn = max(-self.max_turn, min(self.max_turn, turn))
                self.motor.move(self.base_speed, turn)
                self.last_detection_time = time.time()

                # Draw green contours on black road areas
                if self.show_display:
                    # Draw green contour on the detected black road
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    # Draw white circle at center point
                    cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

                # Store frame AFTER drawing so the drawn version is displayed
                self.last_frame = frame.copy()
                if cx >= 120:
                    print("Turn Right")
                    self.motor.move_right(50)
                elif 40 < cx < 120:
                    print("On Track!")
                    self.motor.move_stright(25)
                elif cx <= 40:
                    print("Turn Left")
                    self.motor.move_left(50)

                return True

        # Store frame even when no contours detected
        self.last_frame = frame.copy()

        if time.time() - self.last_detection_time > self.lost_timeout:
            print("[WARN] Line lost - stopping motors.")
            self.motor.stop()

        return False

    def display_frames(self):
        """Display mask and frame windows if display is enabled."""
        if not self.show_display or cv2 is None:
            return

        try:
            if self.last_mask is not None and self.last_frame is not None:
                # Mask already inverted - black areas show as white
                cv2.imshow("Mask", self.last_mask)
                cv2.imshow("Frame", self.last_frame)
                # Non-blocking wait for window updates
                cv2.waitKey(1)
        except Exception as exc:  # pylint: disable=broad-except
            # Silently handle display errors (e.g., headless mode)
            pass

    def stop(self):
        self.motor.stop()

    def cleanup(self):
        self.stop()
        if self.camera:
            self.camera.release()
        if cv2 is not None:
            cv2.destroyAllWindows()
