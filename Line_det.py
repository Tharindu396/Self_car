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

# motor= Motor(2,3,4,17,22,27)   

class PIDController:
    """
    Simple PID controller for line following.
    
    P (Proportional): Responds to current error
    I (Integral): Responds to accumulated past errors
    D (Derivative): Responds to rate of change of error
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_integral=1.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.max_integral = max_integral  # Prevent integral windup
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, error):
        """
        Compute PID output based on error.
        
        Args:
            error: Current error value (typically normalized -1 to 1)
        
        Returns:
            PID correction value
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero
        if dt <= 0.0:
            dt = 0.001
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (accumulated error over time)
        self.integral += error * dt
        # Anti-windup: limit integral to prevent it from growing too large
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term (rate of change of error)
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # Store values for next iteration
        self.last_error = error
        self.last_time = current_time
        
        # Total PID output
        output = p_term + i_term + d_term
        
        return output
    
    def reset(self):
        """Reset the PID controller state."""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class LineFollower:
    """
    Contour-based line follower with PID control.

    - Detects the darkest line on the floor using HSV thresholding
    - Uses PID controller to smoothly adjust motor turn value
    - Optionally provides frames for AprilTag detection
    """

    def __init__(
        self,
        camera_source: Optional[str] = None,  # Deprecated, kept for compatibility
        frame_width: int = 160,
        frame_height: int = 120,
        base_speed: float = 0.45,
        # PID tuning parameters
        kp: float = 0.65,  # Proportional gain (replaces turn_gain)
        ki: float = 0.01,  # Integral gain (helps eliminate steady-state error)
        kd: float = 0.15,  # Derivative gain (reduces oscillation/overshoot)
        max_turn: float = 0.5,
        show_display: bool = True,
    ):
        if not CV2_AVAILABLE or np is None:
            raise RuntimeError("OpenCV + NumPy are required for physical navigation mode.")

        self.base_speed = base_speed
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
        
        # Initialize PID controller
        self.pid = PIDController(kp=kp, ki=ki, kd=kd, max_integral=1.0)

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
        Process one camera frame and adjust motors using PID control.

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
                
                # Calculate normalized error (-1 to 1)
                # Negative error = line is to the left
                # Positive error = line is to the right
                error = (cx - (self.frame_width / 2)) / max(self.frame_width / 2, 1)
                
                # Use PID controller to compute turn value
                turn = -self.pid.compute(error)
                
                # Clamp turn value to maximum allowed
                turn = max(-self.max_turn, min(self.max_turn, turn))
                
                # Move with PID-controlled turn
                self.motor.move(self.base_speed, turn)
                self.last_detection_time = time.time()

                # Draw visualization on frame
                if self.show_display:
                    # Draw green contour on the detected black road
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    # Draw white circle at center point
                    cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                    # Draw center line and error indicator
                    center_x = self.frame_width // 2
                    cv2.line(frame, (center_x, 0), (center_x, self.frame_height), (0, 0, 255), 1)
                    # Display error value on frame
                    cv2.putText(frame, f"Error: {error:.2f}", (10, 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(frame, f"Turn: {turn:.2f}", (10, 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Store frame AFTER drawing
                self.last_frame = frame.copy()
                
                # Optional: Print debug info
                if cx >= 120:
                    status = "Turn Right"
                elif 40 < cx < 120:
                    status = "On Track!"
                else:
                    status = "Turn Left"
                print(f"{status} | Error: {error:.3f} | Turn: {turn:.3f}")

                return True

        # Store frame even when no contours detected
        self.last_frame = frame.copy()

        # Line lost - reset PID to avoid integral windup
        if time.time() - self.last_detection_time > self.lost_timeout:
            print("[WARN] Line lost - stopping motors and resetting PID.")
            self.motor.stop()
            self.pid.reset()

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
        self.pid.reset()

    def cleanup(self):
        self.stop()
        if self.camera:
            self.camera.release()
        if cv2 is not None:
            cv2.destroyAllWindows()