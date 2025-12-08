import cv2
import numpy as np
import sys
from typing import Optional

try:
    from pupil_apriltags import Detector
    APRILTAG_AVAILABLE = True
except ImportError:
    Detector = None
    APRILTAG_AVAILABLE = False

class AprilTagDetector:
    """
    AprilTag detector for junction identification.
    Maps tag IDs to junction nodes and stops motors when detected.
    """

    def __init__(self, motor=None):
        self.motor = motor  # Optional Motor instance to stop on detection
        
        if not APRILTAG_AVAILABLE or Detector is None:
            self.detector = None
            self.enabled = False
            print("[WARN] pupil_apriltags not available. AprilTag detection disabled.")
        else:
            try:
                self.detector = Detector(
                    families="tag25h9",
                    nthreads=4,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=True,
                    decode_sharpening=0.25,
                    debug=False
                )
                self.enabled = True
            except Exception as exc:  # pylint: disable=broad-except
                self.detector = None
                self.enabled = False
                print(f"[WARN] Failed to initialize AprilTag detector: {exc}")

        # Map AprilTag ID to junction node name
        self.tag_to_junction = {
            1: "J1",
            2: "J2",
            3: "J3",
            4: "J4",
        }

    def detect_junction(self, frame) -> Optional[str]:
        """
        Detect AprilTag in frame and return corresponding junction node.
        If a mapped tag is detected, stops the motors (if provided).

        Args:
            frame: OpenCV BGR frame

        Returns:
            Junction node name (e.g., "J1") if tag detected, None otherwise
        """
        if not self.enabled or self.detector is None or frame is None:
            return None

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            if results:
                # Get the first (or largest) detected tag
                tag = results[0]
                tag_id = tag.tag_id

                if tag_id in self.tag_to_junction:
                    junction = self.tag_to_junction[tag_id]
                    
                    # Stop motors when tag is detected
                    if self.motor is not None:
                        try:
                            self.motor.stop()
                            print(f"[APRILTAG] Detected tag {tag_id} -> {junction}. Motors stopped.")
                        except Exception as motor_exc:  # pylint: disable=broad-except
                            print(f"[WARN] Could not stop motor: {motor_exc}")
                    
                    return junction

        except Exception:  # pylint: disable=broad-except
            # Silently fail - don't spam errors during line following
            pass

        return None