"""
Integrated Navigation Controller

Combines graph loading, pathfinding, junction detection, and motion control.
"""

from typing import List, Optional, Tuple, Dict, Any
from enum import Enum
import json

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    CV2_AVAILABLE = False

from graph_loader import Graph
from pathfinder import PathFinder
from junction_detector import JunctionDetector


class NavigationState(Enum):
    """Navigation state machine."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    AT_JUNCTION = "at_junction"
    REACHED_GOAL = "reached_goal"
    ERROR = "error"


class NavigationController:
    """
    Main navigation controller integrating all navigation modules.
    
    Workflow:
    1. Load graph from JSON
    2. Compute path to goal
    3. Follow path with visual confirmation
    4. Detect junctions and execute turns
    5. Reach goal
    """
    
    def __init__(self, graph_path: str, 
                 lane_width_pixels: int = 80,
                 junction_threshold: float = 0.7):
        """
        Initialize navigation controller.
        
        Args:
            graph_path: Path to graph JSON file
            lane_width_pixels: Expected lane width for junction detection
            junction_threshold: Confidence threshold for junction detection
        """
        self.graph = Graph.from_json(graph_path)
        self.pathfinder = PathFinder(self.graph)
        self.junction_detector = JunctionDetector(
            lane_width_pixels=lane_width_pixels,
            junction_sensitivity=junction_threshold
        )
        
        self.state = NavigationState.IDLE
        self.current_path: Optional[List[str]] = None
        self.path_index = 0
        self.current_node: Optional[str] = None
        self.goal_node: Optional[str] = None
        self.last_junction_detection: Optional[str] = None
        self.detection_confidence = 0.0

    def start_navigation(self, start_node: str, goal_node: str) -> bool:
        """
        Start navigation from start to goal.
        
        Args:
            start_node: Starting node name
            goal_node: Goal node name
            
        Returns:
            True if path found, False otherwise
        """
        # Validate nodes
        if start_node not in self.graph.nodes() or goal_node not in self.graph.nodes():
            print(f"[ERROR] Invalid nodes: {start_node} or {goal_node}")
            self.state = NavigationState.ERROR
            return False
        
        # Find path
        self.current_path = self.pathfinder.find_shortest_path(start_node, goal_node)
        
        if not self.current_path:
            print(f"[ERROR] No path found from {start_node} to {goal_node}")
            self.state = NavigationState.ERROR
            return False
        
        # Initialize navigation
        self.current_node = start_node
        self.goal_node = goal_node
        self.path_index = 0
        self.state = NavigationState.NAVIGATING
        
        # Get instructions
        instructions = self.pathfinder.get_path_instructions(self.current_path)
        distance = self.pathfinder.get_total_distance(self.current_path)
        
        print(f"[NAV] Path found: {' -> '.join(self.current_path)}")
        print(f"[NAV] Total distance: {distance:.2f} units")
        print(f"[NAV] Instructions: {len(instructions)} steps")
        
        return True

    def update_position(self, detected_node: Optional[str]) -> Dict:
        """
        Update current position based on detection.
        
        Args:
            detected_node: Node detected by sensor/vision
            
        Returns:
            Status dictionary with current state
        """
        if self.state != NavigationState.NAVIGATING:
            return self._status_dict()
        
        # Update current node if detected
        if detected_node:
            self.current_node = detected_node
            
            # Check if we reached goal
            if detected_node == self.goal_node:
                self.state = NavigationState.REACHED_GOAL
                print(f"[NAV] Reached goal: {self.goal_node}")
                return self._status_dict()
            
            # Update path index
            try:
                self.path_index = self.current_path.index(detected_node)
            except ValueError:
                print(f"[WARN] Detected node {detected_node} not in planned path")
        
        return self._status_dict()

    def detect_junction(self, frame) -> Dict:
        """
        Detect junction from camera frame and return action.
        
        Args:
            frame: OpenCV BGR image frame
            
        Returns:
            Dictionary with junction detection info:
                - 'junction_type': 'straight', 'left', 'right', 'intersection', etc.
                - 'confidence': Confidence level (0-1)
                - 'next_action': Recommended action (e.g., 'turn_left')
                - 'lane_offset': Lateral offset from center
                - 'lane_heading': Heading angle
        """
        if frame is None or not CV2_AVAILABLE:
            return self._empty_detection()
        
        # Detect junction type
        junction_type = self.junction_detector.detect_junction(frame)
        
        # Get lane metrics
        lane_offset, lane_heading = self.junction_detector.detect_lane_offset_and_heading(frame)
        
        # Determine recommended action based on path
        recommended_action = self._get_recommended_action(junction_type)
        
        detection = {
            'junction_type': junction_type or 'unknown',
            'confidence': self.detection_confidence,
            'next_action': recommended_action,
            'lane_offset': lane_offset,
            'lane_heading': lane_heading,
            'current_node': self.current_node,
            'next_node': self._get_next_node()
        }
        
        self.last_junction_detection = junction_type
        return detection

    def _get_recommended_action(self, detected_junction: Optional[str]) -> str:
        """
        Determine recommended action based on path and detection.
        
        Args:
            detected_junction: Detected junction type
            
        Returns:
            Recommended action string
        """
        if self.path_index >= len(self.current_path) - 2:
            return "stop_goal_near"
        
        # Get next turn from path
        current = self.current_path[self.path_index]
        next_node = self.current_path[self.path_index + 1]
        
        if self.path_index + 2 < len(self.current_path):
            future_node = self.current_path[self.path_index + 2]
            turn = self.graph.get_turn_action(current, next_node, future_node)
            return f"turn_{turn}" if turn != "straight" else "continue_straight"
        
        return "continue_straight"

    def _get_next_node(self) -> Optional[str]:
        """Get next node in path."""
        if self.path_index + 1 < len(self.current_path):
            return self.current_path[self.path_index + 1]
        return None

    def _status_dict(self) -> Dict:
        """Get current navigation status."""
        next_node = self._get_next_node()
        return {
            'state': self.state.value,
            'current_node': self.current_node,
            'goal_node': self.goal_node,
            'next_node': next_node,
            'path_progress': f"{self.path_index}/{len(self.current_path) if self.current_path else 0}",
            'remaining_nodes': (len(self.current_path) - self.path_index) if self.current_path else 0
        }

    def _empty_detection(self) -> Dict:
        """Get empty detection response."""
        return {
            'junction_type': 'unknown',
            'confidence': 0.0,
            'next_action': 'no_detection',
            'lane_offset': 0.0,
            'lane_heading': 0.0,
            'current_node': self.current_node,
            'next_node': self._get_next_node()
        }

    def get_current_status(self) -> Dict:
        """Get full navigation status."""
        status = self._status_dict()
        status['last_junction'] = self.last_junction_detection
        status['path'] = self.current_path if self.current_path else []
        return status

    def reset(self):
        """Reset navigation controller."""
        self.state = NavigationState.IDLE
        self.current_path = None
        self.path_index = 0
        self.current_node = None
        self.goal_node = None
        self.last_junction_detection = None
        self.detection_confidence = 0.0
        self.junction_detector.reset()

    def save_current_path(self, filename: str):
        """
        Save current path to JSON file.
        
        Args:
            filename: Output filename
        """
        if not self.current_path:
            print("[WARN] No path to save")
            return
        
        path_data = {
            'start': self.current_path[0],
            'goal': self.goal_node,
            'path': self.current_path,
            'instructions': self.pathfinder.get_path_instructions(self.current_path),
            'total_distance': self.pathfinder.get_total_distance(self.current_path)
        }
        
        with open(filename, 'w') as f:
            json.dump(path_data, f, indent=2)
        
        print(f"[NAV] Path saved to {filename}")
