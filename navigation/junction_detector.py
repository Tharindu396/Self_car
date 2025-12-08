"""
Junction Detection Module

Detects junctions using image processing techniques:
- Lane line pattern analysis
- Cross/Y-junction detection
- No AprilTags required
"""

from typing import Optional, Tuple, List, Dict
import math
import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False


class JunctionDetector:
    """
    Detects junctions by analyzing lane patterns and road structure.
    Uses image processing techniques without AprilTags.
    """
    
    def __init__(self, 
                 lane_width_pixels: int = 80,
                 junction_sensitivity: float = 0.6,
                 min_junction_size: int = 50):
        """
        Initialize junction detector.
        
        Args:
            lane_width_pixels: Expected width of a lane in pixels
            junction_sensitivity: Threshold for junction detection (0-1)
            min_junction_size: Minimum contour area to be considered a junction
        """
        self.lane_width = lane_width_pixels
        self.sensitivity = max(0.1, min(junction_sensitivity, 1.0))
        self.min_size = min_junction_size
        self.junction_history = []  # Track detected junctions over time
        self.max_history = 5

    def detect_junction(self, frame: np.ndarray) -> Optional[str]:
        """
        Detect junction type from camera frame.
        
        Args:
            frame: OpenCV BGR image frame
            
        Returns:
            Junction type: "straight", "left", "right", "cross", or None
        """
        if frame is None or not CV2_AVAILABLE:
            return None
        
        try:
            # Convert to grayscale for processing
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect lane lines and road structure
            junction_type = self._analyze_lane_structure(gray)
            
            # Apply temporal filtering to reduce noise
            junction_type = self._temporal_filter(junction_type)
            
            return junction_type
            
        except Exception as e:
            print(f"[ERROR] Junction detection failed: {e}")
            return None

    def _analyze_lane_structure(self, gray: np.ndarray) -> Optional[str]:
        """
        Analyze grayscale image to detect junction patterns.
        
        Args:
            gray: Grayscale image
            
        Returns:
            Junction type or None
        """
        height, width = gray.shape
        
        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Focus on bottom portion where road markings are clear
        roi_height = int(height * 0.6)
        roi = edges[roi_height:, :]
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(
            roi, 
            rho=1, 
            theta=np.pi/180, 
            threshold=30,
            minLineLength=30,
            maxLineGap=10
        )
        
        if lines is None:
            return "straight"
        
        # Analyze line patterns
        return self._classify_junction_from_lines(lines, roi.shape)

    def _classify_junction_from_lines(self, 
                                      lines: np.ndarray, 
                                      roi_shape: Tuple[int, int]) -> Optional[str]:
        """
        Classify junction type based on detected lines.
        
        Args:
            lines: Detected lines from Hough transform
            roi_shape: Shape of ROI
            
        Returns:
            Junction type
        """
        # Separate lines by angle (vertical vs horizontal)
        vertical_lines = []
        horizontal_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate angle
            if x2 - x1 == 0:
                angle = 90
            else:
                angle = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))
            
            # Normalize angle to 0-90
            if angle > 45:
                angle = 180 - angle
            
            if angle > 30:  # More vertical
                vertical_lines.append(line)
            else:  # More horizontal
                horizontal_lines.append(line)
        
        # Analyze patterns
        num_vertical = len(vertical_lines)
        num_horizontal = len(horizontal_lines)
        
        # Cross junction: multiple lines in all directions
        if num_vertical >= 3 and num_horizontal >= 2:
            return "cross"
        
        # T-junction or Y-junction
        if num_vertical >= 2:
            return "intersection"  # General intersection
        
        # Left/right turn detection based on line distribution
        if len(vertical_lines) > 0:
            # Analyze position of vertical lines
            x_positions = []
            for line in vertical_lines:
                x1, y1, x2, y2 = line[0]
                x_positions.append((x1 + x2) / 2)
            
            if x_positions:
                avg_x = sum(x_positions) / len(x_positions)
                center_x = roi_shape[1] / 2
                
                if avg_x < center_x * 0.8:  # Left biased
                    return "left"
                elif avg_x > center_x * 1.2:  # Right biased
                    return "right"
        
        return "straight"

    def _temporal_filter(self, current_detection: Optional[str]) -> Optional[str]:
        """
        Apply temporal filtering to reduce false positives.
        
        Args:
            current_detection: Current frame's detection
            
        Returns:
            Filtered junction type
        """
        self.junction_history.append(current_detection)
        if len(self.junction_history) > self.max_history:
            self.junction_history.pop(0)
        
        # Require consensus from last N frames
        if len(self.junction_history) < 3:
            return current_detection
        
        # Count occurrences
        history_copy = [h for h in self.junction_history if h is not None]
        if not history_copy:
            return current_detection
        
        # Return most common detection
        most_common = max(set(history_copy), key=history_copy.count)
        
        # Only return if it appears in at least 60% of recent frames
        confidence = history_copy.count(most_common) / len(history_copy)
        if confidence >= 0.6:
            return most_common
        
        return "straight"  # Default to straight if uncertain

    def detect_lane_offset_and_heading(self, frame: np.ndarray) -> Tuple[float, float]:
        """
        Detect lane offset and heading from frame (similar to lane detection model).
        
        Args:
            frame: OpenCV BGR image
            
        Returns:
            (lane_offset, lane_heading) both normalized to [-1, 1]
        """
        if frame is None or not CV2_AVAILABLE:
            return 0.0, 0.0
        
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            
            # Focus on lower portion (where road is visible)
            roi = gray[int(height * 0.5):, :]
            
            # Detect edges
            edges = cv2.Canny(roi, 50, 150)
            
            # Apply Hough line detection
            lines = cv2.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi/180,
                threshold=20,
                minLineLength=20,
                maxLineGap=10
            )
            
            if lines is None:
                return 0.0, 0.0
            
            # Calculate lane offset and heading
            lane_offset = self._calculate_lane_offset(lines, width)
            lane_heading = self._calculate_lane_heading(lines, roi.shape)
            
            return lane_offset, lane_heading
            
        except Exception as e:
            print(f"[ERROR] Lane offset detection failed: {e}")
            return 0.0, 0.0

    def _calculate_lane_offset(self, lines: np.ndarray, frame_width: int) -> float:
        """
        Calculate normalized lane offset from detected lines.
        
        Returns:
            Offset in range [-1, 1] where -1 = far left, 0 = center, 1 = far right
        """
        if lines is None or len(lines) == 0:
            return 0.0
        
        # Get center X positions of all lines
        x_positions = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_positions.append((x1 + x2) / 2)
        
        if not x_positions:
            return 0.0
        
        # Average position
        avg_x = sum(x_positions) / len(x_positions)
        
        # Normalize to [-1, 1]
        center_x = frame_width / 2
        offset = (avg_x - center_x) / (center_x / 2)
        
        return max(-1.0, min(1.0, offset))

    def _calculate_lane_heading(self, lines: np.ndarray, roi_shape: Tuple[int, int]) -> float:
        """
        Calculate normalized lane heading from detected lines.
        
        Returns:
            Heading in range [-1, 1] where -1 = turn left, 0 = straight, 1 = turn right
        """
        if lines is None or len(lines) == 0:
            return 0.0
        
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 - x1 == 0:
                angle = 90
            else:
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
            
            # Normalize to [-90, 90]
            if angle > 90:
                angle -= 180
            elif angle < -90:
                angle += 180
            
            angles.append(angle)
        
        if not angles:
            return 0.0
        
        # Average heading angle
        avg_angle = sum(angles) / len(angles)
        
        # Normalize to [-1, 1]
        heading = avg_angle / 90.0
        
        return max(-1.0, min(1.0, heading))

    def reset(self):
        """Reset junction history for new navigation segment."""
        self.junction_history.clear()
