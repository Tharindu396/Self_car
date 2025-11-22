"""
Main Navigation System

- Uses Dijkstra shortest path from dijikstra_algo.py
- Provides visualization/text simulation modes
- Adds physical navigation mode with contour-based line detection
"""

import time
from typing import List, Optional, Tuple

from dijikstra_algo import dijkstra, graph, detect_start_node_from_camera
from standalone_navigation import StandaloneNavigation, NavigationVisualizer

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


class DummyMotor:
    """Fallback motor used on development machines without GPIO."""

    def move(self, speed: float = 0.0, turn: float = 0.0, t: Optional[float] = None):
        _ = t
        print(f"[SIM MOTOR] speed={speed:.2f}, turn={turn:.2f}")

    def stop(self, t: float = 0.0):
        _ = t
        print("[SIM MOTOR] stop")


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


class AprilTagDetector:
    """
    AprilTag detector for junction identification.
    Maps tag IDs to junction nodes 
    """

    def __init__(self):
        if not APRILTAG_AVAILABLE or apriltag is None:
            self.detector = None
            self.enabled = False
            print("[WARN] pupil_apriltags not available. AprilTag detection disabled.")
        else:
            try:
                self.detector = apriltag.Detector()
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
                    return junction

        except Exception as exc:  # pylint: disable=broad-except
            # Silently fail - don't spam errors during line following
            pass

        return None


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
        self.low_hsv = np.array([0, 0, 0], dtype=np.uint8)
        self.high_hsv = np.array([180, 255, 60], dtype=np.uint8)
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.last_detection_time = time.time()
        self.lost_timeout = 0.75
        self.last_frame = None  # Store last frame for AprilTag detection
        self.show_display = show_display
        self.last_mask = None  # Store mask for display

        if MOTOR_AVAILABLE and Motor is not None:
            self.motor = Motor(2, 3, 4, 17, 22, 27)
        else:
            self.motor = DummyMotor()
            if not MOTOR_AVAILABLE:
                print(f"[WARN] MotorModule unavailable: {MOTOR_IMPORT_ERROR}")

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
        # Invert mask so black areas become white for display
        inverted_mask = cv2.bitwise_not(mask)
        self.last_mask = inverted_mask.copy()
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
                
                return True
        
        # Store frame even when no contours detected
        self.last_frame = frame.copy()

        if time.time() - self.last_detection_time > self.lost_timeout:
            print("[WARN] Line lost - stopping motors.")
            self.motor.stop()

        return False

    def display_frames(self):
        """Display mask and frame windows if display is enabled."""
        if not self.show_display or not CV2_AVAILABLE or cv2 is None:
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
        if CV2_AVAILABLE and cv2 is not None:
            cv2.destroyAllWindows()


class PhysicalNavigator:
    """
    Couples the virtual navigation path with real-world line following.
    Uses AprilTags to identify and verify junction positions.
    """

    def __init__(
        self,
        nav: StandaloneNavigation,
        line_follower: LineFollower,
        route_speed_units: float = 0.35,
        visualizer: Optional[NavigationVisualizer] = None,
    ):
        self.nav = nav
        self.line_follower = line_follower
        self.route_speed_units = max(0.1, route_speed_units)
        self._last_instruction: Optional[str] = None
        self.visualizer = visualizer
        self.tag_detector = AprilTagDetector()
        self._last_detected_junction: Optional[str] = None
        self._junction_detection_cooldown = 2.0  # seconds between detections
        self._last_junction_detection_time = 0.0

    def follow_route(self):
        if not self.line_follower.is_ready:
            print("Line follower is not ready. Cannot start physical navigation.")
            return

        if not self.nav.current_path or len(self.nav.current_path) < 2:
            print("Route is not set. Please compute a path first.")
            return

        print("\n=== Physical Navigation Mode ===")
        print("Following shortest path using contour-based line detection.")
        print("Camera windows (Mask and Frame) and navigation map are displayed.")
        print("Press 'q' in camera window to stop navigation.\n")

        try:
            for idx in range(len(self.nav.current_path) - 1):
                start_node = self.nav.current_path[idx]
                end_node = self.nav.current_path[idx + 1]
                edge_length = self.nav.graph[start_node].get(end_node)
                if edge_length is None or edge_length <= 0:
                    print(f"[WARN] Missing edge data for {start_node}->{end_node}. Skipping segment.")
                    continue

                print(f"[SEGMENT] {start_node} -> {end_node} ({edge_length:.2f} units)")
                self._follow_segment(edge_length)
                print(f"[REACHED] {end_node}")

            print("\n[DONE] Destination reached. Stopping motors.")
        finally:
            self.line_follower.cleanup()

    def _follow_segment(self, edge_length: float):
        target_duration = edge_length / self.route_speed_units
        target_duration = max(target_duration, 0.1)

        start_time = time.time()
        last_time = start_time

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            if dt <= 0:
                continue

            # Process line following and motor control
            self.line_follower.step()
            
            # Display camera windows (mask and frame)
            self.line_follower.display_frames()
            
            # Update navigation position
            still_on_route = self.nav.update_position(self.route_speed_units, dt)
            
            # Check for AprilTag junction detection
            self._check_apriltag_junction()
            
            self._report_junction()
            self._update_visualizer()

            # Check for quit key
            if CV2_AVAILABLE and cv2 is not None:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n[STOP] User pressed 'q' to quit")
                    break

            if not still_on_route:
                break

            if (now - start_time) >= target_duration:
                break

    def _check_apriltag_junction(self):
        """
        Check for AprilTag detection and update navigation position if junction detected.
        Uses cooldown to prevent duplicate detections.
        """
        current_time = time.time()
        
        # Cooldown to prevent rapid re-detection of same tag
        if current_time - self._last_junction_detection_time < self._junction_detection_cooldown:
            return

        frame = self.line_follower.get_last_frame()
        if frame is None:
            return

        detected_junction = self.tag_detector.detect_junction(frame)
        
        if detected_junction and detected_junction != self._last_detected_junction:
            self._last_detected_junction = detected_junction
            self._last_junction_detection_time = current_time
            
            # Verify this junction is in our path
            if self.nav.current_path and detected_junction in self.nav.current_path:
                # Update car position to this junction
                self._update_position_to_junction(detected_junction)
                print(f"[APRILTAG] Detected junction: {detected_junction}")

    def _update_position_to_junction(self, junction_node: str):
        """
        Update navigation position to match detected AprilTag junction.
        This corrects any drift in the virtual position.
        """
        if not self.nav.current_path or junction_node not in self.nav.current_path:
            return

        # Find the junction in the path
        junction_idx = self.nav.current_path.index(junction_node)
        
        # Update car position to be at this junction with 0 progress
        if self.nav.car_position is None:
            return
            
        self.nav.car_position.node = junction_node
        self.nav.car_position.progress = 0.0
        
        # Update heading based on next node in path
        if junction_idx + 1 < len(self.nav.current_path):
            next_node = self.nav.current_path[junction_idx + 1]
            self.nav.car_position.heading = self.nav._calculate_heading(junction_node, next_node)
        
        print(f"[POSITION CORRECTED] Car position updated to {junction_node}")

    def _report_junction(self):
        junction = self.nav.get_junction_decision()
        if junction and junction.instruction != self._last_instruction:
            print(f"[JUNCTION] {junction.instruction} ({junction.direction})")
            self._last_instruction = junction.instruction

    def _update_visualizer(self):
        if self.visualizer:
            try:
                self.visualizer.update_and_draw()
            except Exception as exc:  # pylint: disable=broad-except
                print(f"[WARN] Visualizer update failed: {exc}")
                self.visualizer = None


def main():
    """Main navigation function"""
    print("=== RPI Car Navigation System ===\n")
    print("This system uses:")
    print("  - dijkstra_algo.py for pathfinding")
    print("  - OCR to detect start position")
    print("  - Real-time position tracking")
    print("  - Junction decision making\n")
    
    # Initialize navigation
    nav = StandaloneNavigation()
    
    # Option 1: Use OCR to detect start node
    print("Step 1: Detecting start position...")
    start = detect_start_node_from_camera()
    
    # Option 2: Manual input if OCR fails
    if start is None or start not in graph:
        print("\nOCR detection failed or invalid. Using manual input.")
        start = input("Enter start node (e.g., A, B, J1): ").strip().upper()
    
    # Get destination
    print("\nStep 2: Enter destination")
    goal = input("Enter destination node (e.g., D, F, J4): ").strip().upper()
    
    # Set route
    print("\nStep 3: Calculating route...")
    if not nav.set_route(start, goal):
        print("Failed to set route. Exiting.")
        return
    
    # Choose mode
    print("\nStep 4: Choose run mode")
    print("  1. Text-only (console output)")
    print("  2. Visualization (Uber-style map)")
    print("  3. Physical car (line detection + motors)")
    mode = input("Enter choice (1, 2, or 3): ").strip()

    if mode == "3":
        if run_physical_navigation(nav):
            return
        print("Physical mode unavailable. Falling back to text-only mode.\n")
        mode = "1"
    
    if mode == "2":
        # Visualization mode
        try:
            viz = NavigationVisualizer(nav)
            print("\nVisualization window opened.")
            print("The window shows:")
            print("  - Blue line: Planned route")
            print("  - Red triangle: Your car position")
            print("  - Green circle: Destination")
            print("  - Orange circles: Junctions")
            print("  - Yellow boxes: Junction instructions")
            print("\nClose the window to stop.\n")
            
            # Simulate car movement
            speed = 0.5  # units per second
            running = True
            
            try:
                while running:
                    still_on_route = nav.update_position(speed, dt=0.1)
                    
                    # Get junction decision
                    junction = nav.get_junction_decision()
                    if junction:
                        print(f"[JUNCTION] {junction.instruction}")
                    
                    # Update visualization
                    viz.update_and_draw()
                    
                    if not still_on_route:
                        print("\n[DONE] Reached destination!")
                        # Keep showing final state
                        for _ in range(20):
                            viz.update_and_draw()
                            time.sleep(0.1)
                        break
                    
                    time.sleep(0.1)
                    
            except KeyboardInterrupt:
                print("\nStopped by user")
            except Exception as loop_exc:  # catch other runtime errors inside the loop
                print(f"[WARN] Visualization loop error: {loop_exc}")
            finally:
                import matplotlib.pyplot as plt
                plt.close('all')
        except Exception as e:
            print(f"Visualization error: {e}")
            print("Falling back to text-only mode...")
            mode = "1"
    
    if mode == "1":
        # Text-only mode
        print("\n=== Navigation Started (Text Mode) ===\n")
        print("Simulating car movement...\n")
        
        speed = 0.5  # units per second
        
        for i in range(100):
            still_on_route = nav.update_position(speed, dt=0.1)
            
            print(f"Step {i+1}:")
            print(f"  Position: {nav.car_position}")
            print(f"  Remaining distance: {nav.get_remaining_distance():.2f} units")
            print(f"  Speed: {nav.speed:.2f} units/s")
            
            # Check for junction decisions
            junction = nav.get_junction_decision()
            if junction:
                print(f"  [JUNCTION] {junction.instruction}")
                print(f"     Direction: {junction.direction}")
                print(f"     Distance: {junction.distance_to_junction:.2f} units")
            
            print()
            
            if not still_on_route:
                print("[DONE] Reached destination!")
                break
            
            time.sleep(0.1)


def run_physical_navigation(nav: StandaloneNavigation) -> bool:
    """Start physical navigation mode if hardware is ready."""
    try:
        follower = LineFollower()
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return False

    visualizer = None
    try:
        visualizer = NavigationVisualizer(nav)
        print("[INFO] Uber-style map visualization enabled.")
        print("[INFO] Camera windows (Mask and Frame) will be displayed.")
    except Exception as viz_exc:  # pylint: disable=broad-except
        print(f"[WARN] Could not start visualization: {viz_exc}")
        print("[INFO] Camera windows will still be displayed.")

    driver = PhysicalNavigator(nav, follower, visualizer=visualizer)
    driver.follow_route()
    return True

def quick_pathfinding():
    """Quick pathfinding without navigation tracking"""
    print("=== Quick Pathfinding ===\n")
    
    start = input("Enter start node: ").strip().upper()
    goal = input("Enter goal node: ").strip().upper()
    
    path, distance = dijkstra(graph, start, goal)
    
    if path:
        print(f"\nShortest Path: {' -> '.join(path)}")
        print(f"Total Distance: {distance:.2f} units")
    else:
        print(f"\nNo path found from {start} to {goal}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "quick":
        quick_pathfinding()
    else:
        main()
