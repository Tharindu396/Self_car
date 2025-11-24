
from typing import List, Optional, Tuple
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    cv2 = None
    np = None

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    Picamera2 = None
    PICAMERA2_AVAILABLE = False

try:
    from MotorModule import Motor
    MOTOR_AVAILABLE = True
except Exception as motor_exc:  # pylint: disable=broad-except
    Motor = None
    MOTOR_AVAILABLE = False
    MOTOR_IMPORT_ERROR = motor_exc

try:
    import pupil_apriltags as apriltag
    APRILTAG_AVAILABLE = True
except ImportError:
    apriltag = None
    APRILTAG_AVAILABLE = False

class CameraWrapper:
    """
    Camera wrapper that uses picamera2 on Raspberry Pi, falls back to OpenCV VideoCapture.
    Provides a unified interface for both.
    """

    def __init__(self, frame_width: int = 160, frame_height: int = 120):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.picam2 = None
        self.cv2_cap = None
        self.use_picamera2 = False
        self.is_initialized = False

        # Try picamera2 first (Raspberry Pi)
        if PICAMERA2_AVAILABLE and Picamera2 is not None:
            try:
                self.picam2 = Picamera2()
                # Configure camera
                config = self.picam2.create_video_configuration(
                    main={"size": (frame_width, frame_height), "format": "RGB888"}
                )
                self.picam2.configure(config)
                self.picam2.start()
                self.use_picamera2 = True
                self.is_initialized = True
                print(f"[CAMERA] Using picamera2 ({frame_width}x{frame_height})")
                return
            except Exception as exc:  # pylint: disable=broad-except
                print(f"[WARN] picamera2 initialization failed: {exc}")
                if self.picam2:
                    try:
                        self.picam2.close()
                    except Exception:  # pylint: disable=broad-except
                        pass
                    self.picam2 = None

        # Fallback to OpenCV VideoCapture
        if CV2_AVAILABLE and cv2 is not None:
            candidates = [0, 1]
            for src in candidates:
                try:
                    cap = cv2.VideoCapture(src)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
                        # Verify it actually set the resolution
                        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        if actual_width > 0 and actual_height > 0:
                            self.cv2_cap = cap
                            self.use_picamera2 = False
                            self.is_initialized = True
                            print(f"[CAMERA] Using OpenCV VideoCapture device {src} ({actual_width}x{actual_height})")
                            return
                        cap.release()
                except Exception as exc:  # pylint: disable=broad-except
                    print(f"[WARN] OpenCV camera {src} failed: {exc}")
                    continue

        raise RuntimeError("Unable to initialize any camera (picamera2 or OpenCV)")

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Read a frame from the camera.

        Returns:
            Tuple of (success, frame) where frame is BGR format numpy array
        """
        if not self.is_initialized:
            return False, None

        if self.use_picamera2 and self.picam2:
            try:
                # picamera2 returns RGB888 format
                rgb_array = self.picam2.capture_array()
                # Convert RGB to BGR for OpenCV compatibility
                if CV2_AVAILABLE and cv2 is not None:
                    bgr_frame = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
                    return True, bgr_frame
                return True, rgb_array
            except Exception as exc:  # pylint: disable=broad-except
                print(f"[WARN] picamera2 capture failed: {exc}")
                return False, None

        elif self.cv2_cap:
            ret, frame = self.cv2_cap.read()
            return ret, frame

        return False, None

    def is_opened(self) -> bool:
        """Check if camera is opened and ready."""
        if self.use_picamera2:
            return self.picam2 is not None and self.is_initialized
        return self.cv2_cap is not None and self.cv2_cap.isOpened()

    def release(self):
        """Release camera resources."""
        if self.use_picamera2 and self.picam2:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:  # pylint: disable=broad-except
                pass
            self.picam2 = None

        if self.cv2_cap:
            try:
                self.cv2_cap.release()
            except Exception:  # pylint: disable=broad-except
                pass
            self.cv2_cap = None

        self.is_initialized = False