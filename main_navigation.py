"""
Main Navigation System

Uses navigation package for:
- Graph-based pathfinding (Dijkstra)
- Junction detection via visual analysis
- Physical navigation with line following
"""

import time
import sys
import os
from typing import List, Optional, Dict, Tuple

# Add navigation module to path
sys.path.insert(0, os.path.dirname(__file__))

from navigation import (
    Graph,
    NavigationController,
    NavigationState
)

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
    import Line_det
    LINE_DET_AVAILABLE = True
except ImportError:
    Line_det = None
    LINE_DET_AVAILABLE = False


class PhysicalNavigator:
    """
    Couples virtual navigation with physical line following.
    Uses junction detection via image analysis to verify position.
    """

    def __init__(
        self,
        nav_controller: NavigationController,
        line_follower,
        route_speed_units: float = 0.35,
    ):
        """
        Initialize physical navigator.
        
        Args:
            nav_controller: NavigationController instance
            line_follower: LineFollower instance for physical movement
            route_speed_units: Speed for route following
        """
        self.nav_controller = nav_controller
        self.line_follower = line_follower
        self.route_speed_units = max(0.1, route_speed_units)
        self._last_instruction: Optional[str] = None
        self._last_detected_junction: Optional[str] = None
        self._junction_detection_cooldown = 2.0  # seconds
        self._last_junction_detection_time = 0.0

    def follow_route(self):
        """Follow the planned navigation route using physical line detection."""
        if not self.line_follower.is_ready:
            print("[ERROR] Line follower is not ready. Cannot start physical navigation.")
            return False

        if not self.nav_controller.current_path or len(self.nav_controller.current_path) < 2:
            print("[ERROR] Route not set. Please compute a path first.")
            return False

        print("\n=== Physical Navigation Mode ===")
        print("Following planned route using line detection.")
        print(f"Route: {' → '.join(self.nav_controller.current_path)}")
        print("Press 'q' in camera window to stop.\n")

        try:
            # Follow each segment of the path
            for idx in range(len(self.nav_controller.current_path) - 1):
                start_node = self.nav_controller.current_path[idx]
                end_node = self.nav_controller.current_path[idx + 1]

                # Get edge length
                neighbors = self.nav_controller.graph.neighbors(start_node)
                edge_length = neighbors.get(end_node)

                if edge_length is None or edge_length <= 0:
                    print(f"[WARN] Missing edge data for {start_node}→{end_node}. Skipping.")
                    continue

                print(f"[SEGMENT] {start_node} → {end_node} ({edge_length:.2f} units)")
                self._follow_segment(edge_length)
                print(f"[REACHED] {end_node}")

            print("\n[SUCCESS] Navigation route complete!")
            print("[INFO] Continuing line following. Press 'q' to stop.\n")
            self._continue_line_following()

        except KeyboardInterrupt:
            print("\n[STOP] Navigation interrupted by user")
            return False
        finally:
            self.line_follower.cleanup()
        
        return True

    def _follow_segment(self, edge_length: float):
        """
        Follow a single segment of the path.
        
        Args:
            edge_length: Length of the segment to follow
        """
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

            # Perform line following
            self.line_follower.step()

            # Display camera frames
            self.line_follower.display_frames()

            # Check for junction detection
            frame = self.line_follower.get_last_frame()
            if frame is not None:
                detected_junction = self.nav_controller.detect_junction(frame)
                if detected_junction and detected_junction.get('node'):
                    junction_node = detected_junction['node']
                    if junction_node in self.nav_controller.current_path:
                        print(f"[JUNCTION DETECTED] {junction_node}")
                        break

            # Check for quit key
            if CV2_AVAILABLE and cv2 is not None:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    raise KeyboardInterrupt("User quit")

            # Check if segment duration exceeded
            if (now - start_time) >= target_duration:
                break

            time.sleep(0.01)

    def _continue_line_following(self):
        """Continue line following indefinitely after route completion."""
        while True:
            self.line_follower.step()
            self.line_follower.display_frames()

            # Check for quit key
            if CV2_AVAILABLE and cv2 is not None:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n[STOP] User quit line following")
                    break

            time.sleep(0.01)


def main():
    """Main navigation entry point."""
    print("=== RPI Car Navigation System ===\n")
    print("Navigation modes:")
    print("  1. Text simulation (console output)")
    print("  2. Physical car (line detection + motors)")
    print("")

    # Get graph path
    graph_path = input("Enter graph JSON file path [graph.json]: ").strip()
    if not graph_path:
        graph_path = "graph.json"

    # Load graph
    try:
        graph = Graph.from_json(graph_path)
        print(f"✓ Graph loaded: {len(graph.nodes())} nodes")
        print(f"  Available nodes: {', '.join(sorted(graph.nodes())[:10])}")
    except FileNotFoundError:
        print(f"✗ Graph file not found: {graph_path}")
        return
    except Exception as e:
        print(f"✗ Error loading graph: {e}")
        return

    # Get start node
    start = input("Enter start node: ").strip().upper()
    if start not in graph.nodes():
        print(f"✗ Invalid start node: {start}")
        return

    # Get goal node
    goal = input("Enter goal node: ").strip().upper()
    if goal not in graph.nodes():
        print(f"✗ Invalid goal node: {goal}")
        return

    # Choose mode
    print("\nSelect mode:")
    print("  1. Text simulation")
    print("  2. Physical navigation")
    mode = input("Enter choice (1 or 2): ").strip()

    if mode == "2":
        if not run_physical_navigation(graph, start, goal):
            mode = "1"

    if mode == "1":
        run_text_simulation(graph, start, goal)


def run_text_simulation(graph: Graph, start: str, goal: str):
    """Run text-only navigation simulation."""
    print("\n=== Text Simulation Mode ===\n")

    # Initialize navigation
    nav_controller = NavigationController("graph.json")

    # Start navigation
    if not nav_controller.start_navigation(start, goal):
        print("Failed to start navigation.")
        return

    print(f"\nPath: {' → '.join(nav_controller.current_path)}")
    print(f"Total distance: {nav_controller.pathfinder.get_total_distance(nav_controller.current_path):.2f} units\n")

    # Simulate navigation
    for step in range(100):
        status = nav_controller.update_position(nav_controller.current_path[min(step // 10, len(nav_controller.current_path) - 1)])

        print(f"Step {step + 1}:")
        print(f"  State: {nav_controller.state.value}")
        print(f"  Current node: {nav_controller.current_node}")
        print(f"  Path index: {nav_controller.path_index}")

        if nav_controller.state == NavigationState.REACHED_GOAL:
            print("\n✓ Navigation complete!")
            break

        time.sleep(0.2)


def run_physical_navigation(graph: Graph, start: str, goal: str) -> bool:
    """
    Run physical navigation with line detection.
    
    Args:
        graph: Navigation graph
        start: Start node
        goal: Goal node
        
    Returns:
        True if successful, False otherwise
    """
    if not LINE_DET_AVAILABLE or Line_det is None:
        print("✗ Line_det module not available")
        return False

    try:
        line_follower = Line_det.LineFollower()
    except Exception as e:
        print(f"✗ Error initializing line follower: {e}")
        return False

    try:
        # Initialize navigation controller
        nav_controller = NavigationController("graph.json")

        # Start navigation
        if not nav_controller.start_navigation(start, goal):
            print("✗ Failed to start navigation")
            return False

        # Initialize physical navigator
        driver = PhysicalNavigator(nav_controller, line_follower)

        # Follow route
        success = driver.follow_route()
        return success

    except Exception as e:
        print(f"✗ Physical navigation error: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        try:
            line_follower.cleanup()
        except:
            pass


if __name__ == "__main__":
    import sys


    main()