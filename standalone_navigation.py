"""
Standalone Navigation System - Fixed visualization with all nodes visible
"""

import heapq
import json
import os
import time
import math
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from collections import deque

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.patches import Circle, FancyArrowPatch
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


@dataclass
class Position:
    """Represents a position in the road network"""
    node: str
    progress: float  # 0.0 to 1.0
    heading: float  # degrees
    
    def __str__(self):
        return f"Position(node={self.node}, progress={self.progress:.2f}, heading={self.heading:.1f}deg)"


@dataclass
class JunctionDecision:
    """Decision information at a junction"""
    current_node: str
    next_node: str
    direction: str
    distance_to_junction: float
    instruction: str


class StandaloneNavigation:
    """
    Standalone navigation system using Dijkstra algorithm
    Works independently without planner_part, lane_det, or manage.py
    """
    
    def __init__(self, graph_path: str = "graph.json"):
        """Initialize navigation system"""
        self.graph_path = graph_path
        self.graph = self._load_graph()
        self.current_path: Optional[List[str]] = None
        self.path_cost: float = 0.0
        self.car_position: Optional[Position] = None
        self.destination: Optional[str] = None
        self.speed: float = 0.0
        self.last_update_time: float = time.time()
        
    def _load_graph(self) -> Dict:
        """Load road graph from JSON file"""
        _here = os.path.dirname(__file__)
        _graph_path = os.path.join(_here, self.graph_path)
        
        if not os.path.exists(_graph_path):
            raise FileNotFoundError(f"Graph file not found: {_graph_path}")
            
        with open(_graph_path, "r", encoding="utf-8") as f:
            return json.load(f)
    
    def dijkstra(self, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
        """
        Find shortest path using Dijkstra's algorithm
        Same implementation as dijkstra_algo.py
        
        Args:
            start: Starting node
            goal: Destination node
            
        Returns:
            Tuple of (path as list of nodes, total cost)
        """
        if start not in self.graph:
            raise ValueError(f"Start node '{start}' not in graph")
        if goal not in self.graph:
            raise ValueError(f"Goal node '{goal}' not in graph")
        if start == goal:
            return [start], 0.0
            
        pq = [(0, start, [])]
        visited = set()
        
        while pq:
            cost, node, path = heapq.heappop(pq)
            
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]
            
            if node == goal:
                return path, cost
            
            for neighbor, weight in self.graph[node].items():
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))
        
        return None, float("inf")
    
    def set_route(self, start: str, destination: str) -> bool:
        """
        Set a new route from start to destination
        
        Args:
            start: Starting node
            destination: Destination node
            
        Returns:
            True if route found, False otherwise
        """
        path, cost = self.dijkstra(start, destination)
        
        if path is None:
            print(f"No path found from {start} to {destination}")
            return False
            
        self.current_path = path
        self.path_cost = cost
        self.destination = destination
        
        # Initialize car position at start
        self.car_position = Position(
            node=start,
            progress=0.0,
            heading=0.0
        )
        
        print(f"Route set: {' -> '.join(path)}")
        print(f"Total distance: {cost:.2f} units")
        return True
    
    def update_position(self, speed: float, dt: Optional[float] = None) -> bool:
        """
        Update car position based on current speed and time
        
        Args:
            speed: Current speed (units per second)
            dt: Time delta in seconds (auto-calculated if None)
            
        Returns:
            True if still on route, False if reached destination
        """
        if self.car_position is None or self.current_path is None:
            return False
            
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
        else:
            self.last_update_time = time.time()
            
        if dt <= 0:
            return True
            
        self.speed = speed
        distance = speed * dt
        
        current_idx = self._get_current_path_index()
        if current_idx is None or current_idx >= len(self.current_path) - 1:
            self.car_position.progress = 1.0
            return False
            
        current_node = self.current_path[current_idx]
        next_node = self.current_path[current_idx + 1]
        edge_length = self.graph[current_node].get(next_node, 0)
        
        if edge_length == 0:
            return False
            
        remaining_on_edge = (1.0 - self.car_position.progress) * edge_length
        
        if distance >= remaining_on_edge:
            distance -= remaining_on_edge
            self.car_position.node = next_node
            self.car_position.progress = 0.0
            
            if current_idx + 1 < len(self.current_path) - 1:
                next_next = self.current_path[current_idx + 2]
                self.car_position.heading = self._calculate_heading(next_node, next_next)
            else:
                self.car_position.heading = self._calculate_heading(current_node, next_node)
            
            if distance > 0 and current_idx + 1 < len(self.current_path) - 1:
                return self.update_position(speed, distance / speed)
        else:
            self.car_position.progress += distance / edge_length
            self.car_position.heading = self._calculate_heading(current_node, next_node)
        
        return True
    
    def _get_current_path_index(self) -> Optional[int]:
        """Get index of current node in path"""
        if self.car_position is None or self.current_path is None:
            return None
        try:
            return self.current_path.index(self.car_position.node)
        except ValueError:
            return None
    
    def _calculate_heading(self, from_node: str, to_node: str) -> float:
        """Calculate heading angle between two nodes"""
        if from_node.startswith("J") and to_node.startswith("J"):
            return 45.0
        elif from_node.startswith("J") or to_node.startswith("J"):
            return 90.0
        else:
            return 0.0
    
    def get_junction_decision(self) -> Optional[JunctionDecision]:
        """Get decision information when approaching a junction"""
        if self.car_position is None or self.current_path is None:
            return None
            
        current_idx = self._get_current_path_index()
        if current_idx is None or current_idx >= len(self.current_path) - 1:
            return None
            
        current_node = self.car_position.node
        next_node = self.current_path[current_idx + 1]
        
        if not next_node.startswith("J"):
            return None
            
        if current_node.startswith("J"):
            distance_to_junction = 0.0
        else:
            edge_length = self.graph[current_node].get(next_node, 0)
            distance_to_junction = (1.0 - self.car_position.progress) * edge_length
        
        if current_idx + 2 < len(self.current_path):
            junction_next = self.current_path[current_idx + 2]
            direction = self._determine_direction(next_node, junction_next)
        else:
            direction = "straight"
        
        instruction = self._generate_instruction(direction, distance_to_junction)
        
        return JunctionDecision(
            current_node=current_node,
            next_node=next_node,
            direction=direction,
            distance_to_junction=distance_to_junction,
            instruction=instruction
        )
    
    def _determine_direction(self, junction: str, next_node: str) -> str:
        """Determine turning direction at junction"""
        if junction == "J1":
            if next_node in ["A", "E"]:
                return "back"
            elif next_node == "J3":
                return "right"
            elif next_node == "J2":
                return "straight"
        elif junction == "J2":
            if next_node in ["B", "G"]:
                return "back"
            elif next_node == "J4":
                return "right"
            elif next_node == "J1":
                return "straight"
        elif junction == "J3":
            if next_node in ["C", "G"]:
                return "back"
            elif next_node == "J4":
                return "straight"
            elif next_node == "J1":
                return "left"
        elif junction == "J4":
            if next_node in ["D", "F"]:
                return "back"
            elif next_node == "J3":
                return "straight"
            elif next_node == "J2":
                return "left"
        
        return "straight"
    
    def _generate_instruction(self, direction: str, distance: float) -> str:
        """Generate human-readable navigation instruction"""
        if distance < 0.1:
            return f"At junction: Turn {direction}"
        elif distance < 0.5:
            return f"In {distance:.1f} units: Turn {direction}"
        else:
            return f"In {distance:.1f} units: Prepare to turn {direction}"
    
    def get_next_waypoint(self) -> Optional[str]:
        """Get the next waypoint node in the path"""
        if self.current_path is None or self.car_position is None:
            return None
            
        current_idx = self._get_current_path_index()
        if current_idx is None or current_idx >= len(self.current_path) - 1:
            return None
            
        return self.current_path[current_idx + 1]
    
    def get_remaining_distance(self) -> float:
        """Calculate remaining distance to destination"""
        if self.car_position is None or self.current_path is None:
            return 0.0
            
        current_idx = self._get_current_path_index()
        if current_idx is None:
            return 0.0
            
        remaining = 0.0
        
        if current_idx < len(self.current_path) - 1:
            current_node = self.current_path[current_idx]
            next_node = self.current_path[current_idx + 1]
            edge_length = self.graph[current_node].get(next_node, 0)
            remaining += (1.0 - self.car_position.progress) * edge_length
        
        for i in range(current_idx + 1, len(self.current_path) - 1):
            node1 = self.current_path[i]
            node2 = self.current_path[i + 1]
            remaining += self.graph[node1].get(node2, 0)
        
        return remaining


class NavigationVisualizer:
    """Visualizes navigation with Uber-style path display """
    
    def __init__(self, nav: StandaloneNavigation):
        self.nav = nav
        self.fig = None
        self.ax = None
        
        if not MATPLOTLIB_AVAILABLE:
            print("Matplotlib not available. Visualization disabled.")
            return
            
        self._setup_plot()
    
    def _setup_plot(self):
        """Setup matplotlib figure and axes with proper sizing"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        # Increased figure size for better visibility
        self.fig, self.ax = plt.subplots(figsize=(14, 16))
        self.ax.set_aspect('equal')
        self.ax.set_title("RPI Car Navigation - Real-time Tracking", fontsize=16, fontweight='bold')
        self.ax.set_xlabel("X Position", fontsize=12)
        self.ax.set_ylabel("Y Position", fontsize=12)
        self.ax.grid(True, alpha=0.3)
    
    def _get_node_coordinates(self) -> Dict[str, Tuple[float, float]]:
        """Generate coordinates for nodes in the graph"""
        coords = {}
        coords["A"] = (0, 4)
        coords["E"] = (0, 0)
        coords["B"] = (0, 10)
        coords["C"] = (4, 10)
        coords["D"] = (4, 4)
        coords["F"] = (4, 0)
        coords["G"] = (2, 11)
        coords["J1"] = (0, 6)
        coords["J2"] = (0, 9)
        coords["J3"] = (4, 9)
        coords["J4"] = (4, 6)
        return coords
    
    def _interpolate_position(self, pos: Position, coords: Dict[str, Tuple[float, float]]) -> Tuple[float, float]:
        """Interpolate car position along edge"""
        if pos.node not in coords:
            return (0, 0)
            
        current_idx = self.nav._get_current_path_index()
        if current_idx is None or current_idx >= len(self.nav.current_path) - 1:
            return coords[pos.node]
            
        current_node = self.nav.current_path[current_idx]
        if current_idx + 1 < len(self.nav.current_path):
            next_node = self.nav.current_path[current_idx + 1]
            
            if current_node in coords and next_node in coords:
                x1, y1 = coords[current_node]
                x2, y2 = coords[next_node]
                x = x1 + (x2 - x1) * pos.progress
                y = y1 + (y2 - y1) * pos.progress
                return (x, y)
        
        return coords[pos.node]
    
    def draw_frame(self):
        """Draw a single frame of the visualization"""
        if not MATPLOTLIB_AVAILABLE or self.ax is None:
            return
            
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.set_title("RPI Car Navigation - Real-time Tracking", fontsize=16, fontweight='bold')
        self.ax.set_xlabel("X Position", fontsize=12)
        self.ax.set_ylabel("Y Position", fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        coords = self._get_node_coordinates()
        
        # Draw all edges
        for node, neighbors in self.nav.graph.items():
            if node in coords:
                x1, y1 = coords[node]
                for neighbor, weight in neighbors.items():
                    if neighbor in coords:
                        x2, y2 = coords[neighbor]
                        self.ax.plot([x1, x2], [y1, y2], 'gray', linewidth=2, alpha=0.3, zorder=1)
        
        # Draw planned path (blue line - Uber style)
        if self.nav.current_path:
            path_coords = []
            for node in self.nav.current_path:
                if node in coords:
                    path_coords.append(coords[node])
            
            if len(path_coords) > 1:
                path_x = [c[0] for c in path_coords]
                path_y = [c[1] for c in path_coords]
                self.ax.plot(path_x, path_y, 'b-', linewidth=5, alpha=0.6, 
                           label='Planned Route', zorder=2)
        
        # Draw nodes with larger sizes
        for node, (x, y) in coords.items():
            if node.startswith("J"):
                circle = Circle((x, y), 0.35, color='orange', zorder=4)
                self.ax.add_patch(circle)
                self.ax.text(x, y, node, ha='center', va='center', 
                           fontsize=11, fontweight='bold', zorder=5)
            else:
                circle = Circle((x, y), 0.2, color='gray', zorder=4)
                self.ax.add_patch(circle)
                self.ax.text(x, y, node, ha='center', va='center', 
                           fontsize=9, fontweight='bold', zorder=5)
        
        # Draw destination
        if self.nav.destination and self.nav.destination in coords:
            rx, ry = coords[self.nav.destination]
            rider_circle = Circle((rx, ry), 0.45, color='green', 
                                alpha=0.7, zorder=6)
            self.ax.add_patch(rider_circle)
            self.ax.text(rx, ry, 'DEST', ha='center', va='center',
                        fontsize=10, fontweight='bold', color='white', zorder=7)
        
        # Draw car position with larger size
        if self.nav.car_position:
            car_x, car_y = self._interpolate_position(self.nav.car_position, coords)
            
            if NUMPY_AVAILABLE:
                heading_rad = np.radians(self.nav.car_position.heading)
                car_size = 0.35
                
                triangle_x = [
                    car_x + car_size * np.cos(heading_rad),
                    car_x + car_size * 0.5 * np.cos(heading_rad + 2.5),
                    car_x + car_size * 0.5 * np.cos(heading_rad - 2.5)
                ]
                triangle_y = [
                    car_y + car_size * np.sin(heading_rad),
                    car_y + car_size * 0.5 * np.sin(heading_rad + 2.5),
                    car_y + car_size * 0.5 * np.sin(heading_rad - 2.5)
                ]
            else:
                # Fallback without numpy
                heading_rad = math.radians(self.nav.car_position.heading)
                car_size = 0.35
                
                triangle_x = [
                    car_x + car_size * math.cos(heading_rad),
                    car_x + car_size * 0.5 * math.cos(heading_rad + 2.5),
                    car_x + car_size * 0.5 * math.cos(heading_rad - 2.5)
                ]
                triangle_y = [
                    car_y + car_size * math.sin(heading_rad),
                    car_y + car_size * 0.5 * math.sin(heading_rad + 2.5),
                    car_y + car_size * 0.5 * math.sin(heading_rad - 2.5)
                ]
            
            self.ax.fill(triangle_x, triangle_y, color='red', zorder=8, alpha=0.8)
            self.ax.plot(car_x, car_y, 'ro', markersize=14, zorder=9)
            self.ax.text(car_x, car_y + 0.6, 'CAR', ha='center', va='bottom',
                        fontsize=10, fontweight='bold', color='red', zorder=10)
        
        # Draw junction decision
        junction_decision = self.nav.get_junction_decision()
        if junction_decision:
            if junction_decision.next_node in coords:
                jx, jy = coords[junction_decision.next_node]
                
                if self.nav.car_position:
                    car_x, car_y = self._interpolate_position(self.nav.car_position, coords)
                    
                    arrow_color = {
                        'left': 'blue',
                        'right': 'orange',
                        'straight': 'green',
                        'back': 'red'
                    }.get(junction_decision.direction, 'black')
                    
                    arrow = FancyArrowPatch(
                        (car_x, car_y), (jx, jy),
                        arrowstyle='->', mutation_scale=25,
                        color=arrow_color, linewidth=4, alpha=0.7, zorder=3
                    )
                    self.ax.add_patch(arrow)
                
                self.ax.text(jx, jy + 0.7, junction_decision.instruction,
                           ha='center', va='bottom', fontsize=11,
                           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8),
                           zorder=11, fontweight='bold')
        
        # Add info text
        info_text = []
        if self.nav.car_position:
            info_text.append(f"Car: {self.nav.car_position.node} ({self.nav.car_position.progress:.1%})")
        if self.nav.destination:
            info_text.append(f"Destination: {self.nav.destination}")
        if self.nav.current_path:
            info_text.append(f"Path: {' -> '.join(self.nav.current_path)}")
        info_text.append(f"Remaining: {self.nav.get_remaining_distance():.2f} units")
        info_text.append(f"Speed: {self.nav.speed:.2f} units/s")
        
        self.ax.text(0.02, 0.98, '\n'.join(info_text),
                    transform=self.ax.transAxes, fontsize=10,
                    verticalalignment='top', family='monospace',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
                    zorder=12)
        
        self.ax.legend(loc='upper right', fontsize=10)
        
        # FIXED: Extended axis limits to show ALL nodes including G at y=11
        self.ax.set_xlim(-1, 8)
        self.ax.set_ylim(-1, 14.5)
    
    def update_and_draw(self):
        """Update and draw single frame"""
        if not MATPLOTLIB_AVAILABLE:
            return
        self.draw_frame()
        plt.draw()
        plt.pause(0.01)