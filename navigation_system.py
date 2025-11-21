# """
# Navigation System for RPI Car with Real-time Position Tracking and Visualization
# Provides Uber-style path visualization and junction decision making
# """

# import heapq
# import json
# import os
# import time
# from typing import Dict, List, Optional, Tuple, Set
# import numpy as np
# from dataclasses import dataclass
# from collections import deque

# try:
#     import matplotlib.pyplot as plt
#     import matplotlib.animation as animation
#     from matplotlib.patches import Circle, FancyBboxPatch, FancyArrowPatch
#     MATPLOTLIB_AVAILABLE = True
# except ImportError:
#     MATPLOTLIB_AVAILABLE = False
#     print("Warning: matplotlib not available. Visualization disabled.")


# @dataclass
# class Position:
#     """Represents a position in the road network"""
#     node: str  # Current node (e.g., "A", "J1")
#     progress: float  # Progress along edge (0.0 to 1.0)
#     heading: float  # Heading angle in degrees (0-360)
    
#     def __str__(self):
#         return f"Position(node={self.node}, progress={self.progress:.2f}, heading={self.heading:.1f}deg)"


# @dataclass
# class JunctionDecision:
#     """Decision information at a junction"""
#     current_node: str
#     next_node: str
#     direction: str  # "straight", "left", "right", "back"
#     distance_to_junction: float
#     instruction: str


# class NavigationSystem:
#     """
#     Main navigation system that handles pathfinding, position tracking,
#     and junction decisions using Dijkstra's algorithm.
#     """
    
#     def __init__(self, graph_path: str = "graph.json"):
#         """Initialize navigation system with road graph"""
#         self.graph_path = graph_path
#         self.graph = self._load_graph()
#         self.current_path: Optional[List[str]] = None
#         self.path_cost: float = 0.0
#         self.car_position: Optional[Position] = None
#         self.rider_position: Optional[str] = None  # Destination node
#         self.position_history: deque = deque(maxlen=100)
#         self.speed: float = 0.0  # Current speed (units per second)
#         self.last_update_time: float = time.time()
        
#     def _load_graph(self) -> Dict:
#         """Load road graph from JSON file"""
#         _here = os.path.dirname(__file__)
#         _graph_path = os.path.join(_here, self.graph_path)
        
#         if not os.path.exists(_graph_path):
#             raise FileNotFoundError(f"Graph file not found: {_graph_path}")
            
#         with open(_graph_path, "r", encoding="utf-8") as f:
#             return json.load(f)
    
#     def dijkstra_path(self, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
#         """
#         Find shortest path using Dijkstra's algorithm
        
#         Args:
#             start: Starting node
#             goal: Destination node
            
#         Returns:
#             Tuple of (path as list of nodes, total cost)
#         """
#         if start not in self.graph:
#             raise ValueError(f"Start node '{start}' not in graph")
#         if goal not in self.graph:
#             raise ValueError(f"Goal node '{goal}' not in graph")
#         if start == goal:
#             return [start], 0.0
            
#         # Priority queue: (cost, node, path)
#         pq = [(0, start, [])]
#         visited: Set[str] = set()
#         costs: Dict[str, float] = {start: 0}
#         parents: Dict[str, Optional[str]] = {start: None}
        
#         while pq:
#             cost, node, path = heapq.heappop(pq)
            
#             if node in visited:
#                 continue
                
#             visited.add(node)
#             path = path + [node]
            
#             if node == goal:
#                 # Reconstruct path
#                 full_path = []
#                 current = goal
#                 while current is not None:
#                     full_path.append(current)
#                     current = parents[current]
#                 full_path.reverse()
#                 return full_path, cost
            
#             # Explore neighbors
#             for neighbor, weight in self.graph[node].items():
#                 if neighbor not in visited:
#                     new_cost = cost + weight
#                     if neighbor not in costs or new_cost < costs[neighbor]:
#                         costs[neighbor] = new_cost
#                         parents[neighbor] = node
#                         heapq.heappush(pq, (new_cost, neighbor, path))
        
#         return None, float("inf")
    
#     def set_route(self, start: str, destination: str) -> bool:
#         """
#         Set a new route from start to destination
        
#         Args:
#             start: Starting node
#             destination: Destination node
            
#         Returns:
#             True if route found, False otherwise
#         """
#         path, cost = self.dijkstra_path(start, destination)
        
#         if path is None:
#             print(f"No path found from {start} to {destination}")
#             return False
            
#         self.current_path = path
#         self.path_cost = cost
#         self.rider_position = destination
        
#         # Initialize car position at start
#         self.car_position = Position(
#             node=start,
#             progress=0.0,
#             heading=0.0
#         )
        
#         print(f"Route set: {' -> '.join(path)}")
#         print(f"Total distance: {cost:.2f} units")
#         return True
    
#     def update_position(self, speed: float, dt: Optional[float] = None) -> bool:
#         """
#         Update car position based on current speed and time
        
#         Args:
#             speed: Current speed (units per second)
#             dt: Time delta in seconds (auto-calculated if None)
            
#         Returns:
#             True if still on route, False if reached destination
#         """
#         if self.car_position is None or self.current_path is None:
#             return False
            
#         if dt is None:
#             current_time = time.time()
#             dt = current_time - self.last_update_time
#             self.last_update_time = current_time
#         else:
#             self.last_update_time = time.time()
            
#         if dt <= 0:
#             return True
            
#         self.speed = speed
        
#         # Calculate distance traveled
#         distance = speed * dt
        
#         # Get current edge
#         current_idx = self._get_current_path_index()
#         if current_idx is None or current_idx >= len(self.current_path) - 1:
#             # Reached destination
#             self.car_position.progress = 1.0
#             return False
            
#         current_node = self.current_path[current_idx]
#         next_node = self.current_path[current_idx + 1]
#         edge_length = self.graph[current_node].get(next_node, 0)
        
#         if edge_length == 0:
#             return False
            
#         # Update progress along edge
#         remaining_on_edge = (1.0 - self.car_position.progress) * edge_length
#         distance_remaining = remaining_on_edge
        
#         if distance >= distance_remaining:
#             # Move to next node
#             distance -= distance_remaining
#             self.car_position.node = next_node
#             self.car_position.progress = 0.0
            
#             # Update heading based on next edge
#             if current_idx + 1 < len(self.current_path) - 1:
#                 next_next = self.current_path[current_idx + 2]
#                 self.car_position.heading = self._calculate_heading(
#                     next_node, next_next
#                 )
#             else:
#                 # Approaching destination
#                 self.car_position.heading = self._calculate_heading(
#                     current_node, next_node
#                 )
            
#             # Continue moving if there's remaining distance
#             if distance > 0 and current_idx + 1 < len(self.current_path) - 1:
#                 return self.update_position(speed, distance / speed)
#         else:
#             # Still on current edge
#             self.car_position.progress += distance / edge_length
#             # Update heading
#             self.car_position.heading = self._calculate_heading(
#                 current_node, next_node
#             )
        
#         # Save position history
#         self.position_history.append((
#             time.time(),
#             Position(
#                 node=self.car_position.node,
#                 progress=self.car_position.progress,
#                 heading=self.car_position.heading
#             )
#         ))
        
#         return True
    
#     def _get_current_path_index(self) -> Optional[int]:
#         """Get index of current node in path"""
#         if self.car_position is None or self.current_path is None:
#             return None
#         try:
#             return self.current_path.index(self.car_position.node)
#         except ValueError:
#             return None
    
#     def _calculate_heading(self, from_node: str, to_node: str) -> float:
#         """
#         Calculate heading angle between two nodes
#         Simple implementation - can be enhanced with actual coordinates
#         """
#         # For now, use a simple heuristic based on node names
#         # In a real system, you'd use actual GPS/coordinate data
#         if from_node.startswith("J") and to_node.startswith("J"):
#             # Junction to junction
#             return 45.0
#         elif from_node.startswith("J"):
#             # Junction to road
#             return 90.0
#         elif to_node.startswith("J"):
#             # Road to junction
#             return 90.0
#         else:
#             # Road to road
#             return 0.0
    
#     def get_junction_decision(self) -> Optional[JunctionDecision]:
#         """
#         Get decision information when approaching a junction
        
#         Returns:
#             JunctionDecision if approaching junction, None otherwise
#         """
#         if self.car_position is None or self.current_path is None:
#             return None
            
#         current_idx = self._get_current_path_index()
#         if current_idx is None or current_idx >= len(self.current_path) - 1:
#             return None
            
#         current_node = self.car_position.node
#         next_node = self.current_path[current_idx + 1]
        
#         # Check if next node is a junction
#         if not next_node.startswith("J"):
#             return None
            
#         # Check if we're close to the junction
#         if current_node.startswith("J"):
#             # Already at junction
#             distance_to_junction = 0.0
#         else:
#             # Calculate distance to junction
#             edge_length = self.graph[current_node].get(next_node, 0)
#             distance_to_junction = (1.0 - self.car_position.progress) * edge_length
        
#         # Determine direction
#         if current_idx + 2 < len(self.current_path):
#             junction_next = self.current_path[current_idx + 2]
#             direction = self._determine_direction(next_node, junction_next)
#         else:
#             direction = "straight"
        
#         # Generate instruction
#         instruction = self._generate_instruction(direction, distance_to_junction)
        
#         return JunctionDecision(
#             current_node=current_node,
#             next_node=next_node,
#             direction=direction,
#             distance_to_junction=distance_to_junction,
#             instruction=instruction
#         )
    
#     def _determine_direction(self, junction: str, next_node: str) -> str:
#         """
#         Determine turning direction at junction
#         Simplified - can be enhanced with actual road geometry
#         """
#         # Simple heuristic based on node relationships
#         # In real system, use actual road angles
#         if junction == "J1":
#             if next_node in ["A", "E"]:
#                 return "back"
#             elif next_node == "J3":
#                 return "right"
#             elif next_node == "J2":
#                 return "straight"
#         elif junction == "J2":
#             if next_node in ["B", "G"]:
#                 return "back"
#             elif next_node == "J4":
#                 return "right"
#             elif next_node == "J1":
#                 return "straight"
#         elif junction == "J3":
#             if next_node in ["C", "G"]:
#                 return "back"
#             elif next_node == "J4":
#                 return "straight"
#             elif next_node == "J1":
#                 return "left"
#         elif junction == "J4":
#             if next_node in ["D", "F"]:
#                 return "back"
#             elif next_node == "J3":
#                 return "straight"
#             elif next_node == "J2":
#                 return "left"
        
#         return "straight"
    
#     def _generate_instruction(self, direction: str, distance: float) -> str:
#         """Generate human-readable navigation instruction"""
#         if distance < 0.1:
#             return f"At junction: Turn {direction}"
#         elif distance < 0.5:
#             return f"In {distance:.1f} units: Turn {direction}"
#         else:
#             return f"In {distance:.1f} units: Prepare to turn {direction}"
    
#     def get_next_waypoint(self) -> Optional[str]:
#         """Get the next waypoint node in the path"""
#         if self.current_path is None or self.car_position is None:
#             return None
            
#         current_idx = self._get_current_path_index()
#         if current_idx is None or current_idx >= len(self.current_path) - 1:
#             return None
            
#         return self.current_path[current_idx + 1]
    
#     def get_remaining_distance(self) -> float:
#         """Calculate remaining distance to destination"""
#         if self.car_position is None or self.current_path is None:
#             return 0.0
            
#         current_idx = self._get_current_path_index()
#         if current_idx is None:
#             return 0.0
            
#         remaining = 0.0
        
#         # Distance remaining on current edge
#         if current_idx < len(self.current_path) - 1:
#             current_node = self.current_path[current_idx]
#             next_node = self.current_path[current_idx + 1]
#             edge_length = self.graph[current_node].get(next_node, 0)
#             remaining += (1.0 - self.car_position.progress) * edge_length
        
#         # Distance for remaining edges
#         for i in range(current_idx + 1, len(self.current_path) - 1):
#             node1 = self.current_path[i]
#             node2 = self.current_path[i + 1]
#             remaining += self.graph[node1].get(node2, 0)
        
#         return remaining


# class NavigationVisualizer:
#     """
#     Visualizes the navigation system with Uber-style path display
#     Shows car position, rider position, and route path
#     """
    
#     def __init__(self, nav_system: NavigationSystem):
#         self.nav = nav_system
#         self.fig = None
#         self.ax = None
#         self.animation = None
        
#         if not MATPLOTLIB_AVAILABLE:
#             print("Matplotlib not available. Visualization disabled.")
#             return
            
#         self._setup_plot()
    
#     def _setup_plot(self):
#         """Setup matplotlib figure and axes"""
#         if not MATPLOTLIB_AVAILABLE:
#             return
            
#         self.fig, self.ax = plt.subplots(figsize=(12, 10))
#         self.ax.set_aspect('equal')
#         self.ax.set_title("RPI Car Navigation - Real-time Tracking", fontsize=14, fontweight='bold')
#         self.ax.set_xlabel("X Position")
#         self.ax.set_ylabel("Y Position")
#         self.ax.grid(True, alpha=0.3)
    
#     def _get_node_coordinates(self) -> Dict[str, Tuple[float, float]]:
#         """
#         Generate coordinates for nodes in the graph
#         Creates a simple layout - can be replaced with actual GPS coordinates
#         """
#         coords = {}
        
#         # Create a grid layout
#         # Roads A-G on edges, Junctions J1-J4 in center
#         coords["A"] = (0, 3)
#         coords["E"] = (0, 0)
#         coords["B"] = (3, 6)
#         coords["C"] = (6, 6)
#         coords["D"] = (9, 3)
#         coords["F"] = (9, 0)
#         coords["G"] = (4.5, 6)
        
#         # Junctions in a square
#         coords["J1"] = (1.5, 1.5)
#         coords["J2"] = (4.5, 1.5)
#         coords["J3"] = (7.5, 1.5)
#         coords["J4"] = (4.5, 4.5)
        
#         return coords
    
#     def _interpolate_position(self, pos: Position, coords: Dict[str, Tuple[float, float]]) -> Tuple[float, float]:
#         """Interpolate car position along edge"""
#         if pos.node not in coords:
#             return (0, 0)
            
#         current_idx = self.nav._get_current_path_index()
#         if current_idx is None or current_idx >= len(self.nav.current_path) - 1:
#             return coords[pos.node]
            
#         current_node = self.nav.current_path[current_idx]
#         if current_idx + 1 < len(self.nav.current_path):
#             next_node = self.nav.current_path[current_idx + 1]
            
#             if current_node in coords and next_node in coords:
#                 x1, y1 = coords[current_node]
#                 x2, y2 = coords[next_node]
                
#                 # Interpolate
#                 x = x1 + (x2 - x1) * pos.progress
#                 y = y1 + (y2 - y1) * pos.progress
#                 return (x, y)
        
#         return coords[pos.node]
    
#     def draw_frame(self, frame_num: int = 0):
#         """Draw a single frame of the visualization"""
#         if not MATPLOTLIB_AVAILABLE or self.ax is None:
#             return
            
#         self.ax.clear()
#         self.ax.set_aspect('equal')
#         self.ax.set_title("RPI Car Navigation - Real-time Tracking", fontsize=14, fontweight='bold')
#         self.ax.set_xlabel("X Position")
#         self.ax.set_ylabel("Y Position")
#         self.ax.grid(True, alpha=0.3)
        
#         coords = self._get_node_coordinates()
        
#         # Draw all edges (roads)
#         for node, neighbors in self.nav.graph.items():
#             if node in coords:
#                 x1, y1 = coords[node]
#                 for neighbor, weight in neighbors.items():
#                     if neighbor in coords:
#                         x2, y2 = coords[neighbor]
#                         # Draw road
#                         self.ax.plot([x1, x2], [y1, y2], 'gray', linewidth=2, alpha=0.3, zorder=1)
        
#         # Draw planned path (Uber-style blue line)
#         if self.nav.current_path:
#             path_coords = []
#             for node in self.nav.current_path:
#                 if node in coords:
#                     path_coords.append(coords[node])
            
#             if len(path_coords) > 1:
#                 path_x = [c[0] for c in path_coords]
#                 path_y = [c[1] for c in path_coords]
#                 self.ax.plot(path_x, path_y, 'b-', linewidth=4, alpha=0.6, 
#                            label='Planned Route', zorder=2)
        
#         # Draw nodes
#         for node, (x, y) in coords.items():
#             if node.startswith("J"):
#                 # Junction - larger circle
#                 circle = Circle((x, y), 0.3, color='orange', zorder=4)
#                 self.ax.add_patch(circle)
#                 self.ax.text(x, y, node, ha='center', va='center', 
#                            fontsize=10, fontweight='bold', zorder=5)
#             else:
#                 # Road node - smaller circle
#                 circle = Circle((x, y), 0.15, color='gray', zorder=4)
#                 self.ax.add_patch(circle)
#                 self.ax.text(x, y, node, ha='center', va='center', 
#                            fontsize=8, zorder=5)
        
#         # Draw rider position (destination)
#         if self.nav.rider_position and self.nav.rider_position in coords:
#             rx, ry = coords[self.nav.rider_position]
#             rider_circle = Circle((rx, ry), 0.4, color='green', 
#                                 alpha=0.7, zorder=6)
#             self.ax.add_patch(rider_circle)
#             self.ax.text(rx, ry, 'RIDER', ha='center', va='center',
#                         fontsize=9, fontweight='bold', color='white', zorder=7)
        
#         # Draw car position
#         if self.nav.car_position:
#             car_x, car_y = self._interpolate_position(self.nav.car_position, coords)
            
#             # Car icon (triangle pointing in heading direction)
#             heading_rad = np.radians(self.nav.car_position.heading)
#             car_size = 0.3
            
#             # Create triangle for car
#             triangle_x = [
#                 car_x + car_size * np.cos(heading_rad),
#                 car_x + car_size * 0.5 * np.cos(heading_rad + 2.5),
#                 car_x + car_size * 0.5 * np.cos(heading_rad - 2.5)
#             ]
#             triangle_y = [
#                 car_y + car_size * np.sin(heading_rad),
#                 car_y + car_size * 0.5 * np.sin(heading_rad + 2.5),
#                 car_y + car_size * 0.5 * np.sin(heading_rad - 2.5)
#             ]
            
#             self.ax.fill(triangle_x, triangle_y, color='red', zorder=8, alpha=0.8)
#             self.ax.plot(car_x, car_y, 'ro', markersize=12, zorder=9)
#             self.ax.text(car_x, car_y + 0.5, 'CAR', ha='center', va='bottom',
#                         fontsize=9, fontweight='bold', color='red', zorder=10)
        
#         # Draw junction decision if approaching
#         junction_decision = self.nav.get_junction_decision()
#         if junction_decision:
#             if junction_decision.next_node in coords:
#                 jx, jy = coords[junction_decision.next_node]
                
#                 # Draw arrow indicating direction
#                 if self.nav.car_position:
#                     car_x, car_y = self._interpolate_position(self.nav.car_position, coords)
                    
#                     # Arrow color based on direction
#                     arrow_color = {
#                         'left': 'blue',
#                         'right': 'orange',
#                         'straight': 'green',
#                         'back': 'red'
#                     }.get(junction_decision.direction, 'black')
                    
#                     arrow = FancyArrowPatch(
#                         (car_x, car_y), (jx, jy),
#                         arrowstyle='->', mutation_scale=20,
#                         color=arrow_color, linewidth=3, alpha=0.7, zorder=3
#                     )
#                     self.ax.add_patch(arrow)
                
#                 # Draw instruction text
#                 self.ax.text(jx, jy + 0.6, junction_decision.instruction,
#                            ha='center', va='bottom', fontsize=10,
#                            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8),
#                            zorder=11, fontweight='bold')
        
#         # Add info text
#         info_text = []
#         if self.nav.car_position:
#             info_text.append(f"Car: {self.nav.car_position.node} ({self.nav.car_position.progress:.1%})")
#         if self.nav.rider_position:
#             info_text.append(f"Destination: {self.nav.rider_position}")
#         if self.nav.current_path:
#             info_text.append(f"Path: {' -> '.join(self.nav.current_path)}")
#         info_text.append(f"Remaining: {self.nav.get_remaining_distance():.2f} units")
#         info_text.append(f"Speed: {self.nav.speed:.2f} units/s")
        
#         self.ax.text(0.02, 0.98, '\n'.join(info_text),
#                     transform=self.ax.transAxes, fontsize=9,
#                     verticalalignment='top', family='monospace',
#                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
#                     zorder=12)
        
#         self.ax.legend(loc='upper right')
#         self.ax.set_xlim(-1, 10)
#         self.ax.set_ylim(-1, 7)
    
#     def start_animation(self, interval: int = 100):
#         """Start animated visualization"""
#         if not MATPLOTLIB_AVAILABLE:
#             print("Cannot start animation: matplotlib not available")
#             return
            
#         if self.animation is not None:
#             self.animation.event_source.stop()
        
#         self.animation = animation.FuncAnimation(
#             self.fig, lambda x: self.draw_frame(x),
#             interval=interval, blit=False
#         )
#         plt.show()
    
#     def update_and_draw(self):
#         """Update and draw single frame (for non-animated use)"""
#         if not MATPLOTLIB_AVAILABLE:
#             return
#         self.draw_frame()
#         plt.draw()
#         plt.pause(0.01)


# def demo_navigation():
#     """Demo function showing navigation system in action"""
#     print("=== RPI Car Navigation System Demo ===\n")
    
#     # Initialize navigation
#     nav = NavigationSystem()
    
#     # Set route from A to D
#     print("Setting route from A to D...")
#     nav.set_route("A", "D")
    
#     # Create visualizer
#     if MATPLOTLIB_AVAILABLE:
#         viz = NavigationVisualizer(nav)
#         print("\nStarting visualization...")
#         print("Close the plot window to stop.\n")
        
#         # Simulate car movement
#         speed = 0.5  # units per second
#         running = True
        
#         try:
#             while running:
#                 # Update position
#                 still_on_route = nav.update_position(speed)
                
#                 # Check junction decision
#                 junction_decision = nav.get_junction_decision()
#                 if junction_decision:
#                     print(f"[JUNCTION] {junction_decision.instruction}")
#                     print(f"   Distance: {junction_decision.distance_to_junction:.2f} units")
#                     print(f"   Direction: {junction_decision.direction}")
                
#                 # Update visualization
#                 viz.update_and_draw()
                
#                 if not still_on_route:
#                     print("\n[DONE] Reached destination!")
#                     break
                
#                 time.sleep(0.1)  # Update every 100ms
                
#         except KeyboardInterrupt:
#             print("\nStopped by user")
#     else:
#         print("Matplotlib not available. Running text-only demo...")
#         # Text-only simulation
#         speed = 0.5
#         for i in range(50):
#             nav.update_position(speed, 0.1)
#             junction_decision = nav.get_junction_decision()
#             if junction_decision:
#                 print(f"[JUNCTION] {junction_decision.instruction}")
#             if nav.car_position:
#                 print(f"Car at: {nav.car_position}")
#             time.sleep(0.1)


# if __name__ == "__main__":
#     demo_navigation()

