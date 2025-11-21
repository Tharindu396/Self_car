# """
# Improved Navigation Synchronization
# Provides better integration with actual car sensors and movement
# """

# from typing import Dict, List, Optional, Tuple, Callable
# from navigation_system import NavigationSystem, Position, JunctionDecision
# from planner_part import LaneChangePlanner
# import time


# class SpeedSensor:
#     """
#     Abstract base class for speed sensors
#     Implement this with your actual sensor hardware
#     """
    
#     def get_speed(self) -> float:
#         """
#         Get current speed in units per second
        
#         Returns:
#             Speed in units/second (must match graph.json units)
#         """
#         raise NotImplementedError("Subclass must implement get_speed()")
    
#     def is_available(self) -> bool:
#         """Check if sensor is available"""
#         return True


# class ThrottleBasedSpeedSensor(SpeedSensor):
#     """
#     Estimates speed from throttle value
#     Simple fallback when no real sensor available
#     """
    
#     def __init__(self, throttle_to_speed_factor: float = 2.0):
#         """
#         Args:
#             throttle_to_speed_factor: Conversion factor (throttle * factor = speed)
#         """
#         self.factor = throttle_to_speed_factor
#         self.last_throttle = 0.0
#         self.current_speed = 0.0
        
#     def update_throttle(self, throttle: float):
#         """Update with latest throttle value"""
#         self.last_throttle = throttle
#         self.current_speed = throttle * self.factor
        
#     def get_speed(self) -> float:
#         return self.current_speed


# class OdometrySensor:
#     """
#     Abstract class for odometry (distance traveled)
#     Use with wheel encoders or IMU
#     """
    
#     def get_distance_traveled(self) -> float:
#         """
#         Get distance traveled since last call (in units)
        
#         Returns:
#             Distance in units matching graph.json
#         """
#         raise NotImplementedError("Subclass must implement get_distance_traveled()")
    
#     def reset(self):
#         """Reset odometry counter"""
#         pass


# class ImprovedNavigationController:
#     """
#     Improved navigation controller with better sensor integration
#     and synchronization with actual car movement
#     """
    
#     def __init__(
#         self,
#         graph_path: str = "graph.json",
#         max_throttle: float = 0.25,
#         safety_gap_m: float = 1.2,
#         speed_sensor: Optional[SpeedSensor] = None,
#         odometry_sensor: Optional[OdometrySensor] = None,
#         throttle_to_speed_factor: float = 2.0,
#     ):
#         """
#         Initialize improved navigation controller
        
#         Args:
#             graph_path: Path to graph.json file
#             max_throttle: Maximum throttle value
#             safety_gap_m: Safety gap for lane changes
#             speed_sensor: Optional speed sensor (if None, uses throttle estimation)
#             odometry_sensor: Optional odometry sensor for direct distance measurement
#             throttle_to_speed_factor: Factor to convert throttle to speed
#         """
#         self.nav_system = NavigationSystem(graph_path)
#         self.lane_planner = LaneChangePlanner(
#             max_throttle=max_throttle,
#             safety_gap_m=safety_gap_m
#         )
        
#         # Sensor setup
#         self.speed_sensor = speed_sensor
#         self.odometry_sensor = odometry_sensor
        
#         # Fallback to throttle-based if no sensor
#         if self.speed_sensor is None:
#             self.speed_sensor = ThrottleBasedSpeedSensor(throttle_to_speed_factor)
        
#         self.is_navigating: bool = False
#         self.last_update_time: float = time.time()
#         self.last_throttle: float = 0.0
        
#     def start_navigation(self, start_node: str, destination_node: str) -> bool:
#         """Start navigation to destination"""
#         success = self.nav_system.set_route(start_node, destination_node)
#         if success:
#             self.is_navigating = True
#             self.last_update_time = time.time()
#             if self.odometry_sensor:
#                 self.odometry_sensor.reset()
#             print(f"[OK] Navigation started: {start_node} -> {destination_node}")
#         return success
    
#     def update(
#         self,
#         lane_offset: float,
#         lane_heading: float,
#         detections: Optional[List[Dict]],
#         dt: Optional[float] = None,
#     ) -> Tuple[float, float, Dict]:
#         """
#         Main update function with improved synchronization
        
#         Args:
#             lane_offset: Current lane offset
#             lane_heading: Current lane heading
#             detections: Object detections
#             dt: Time delta (auto-calculated if None)
            
#         Returns:
#             Tuple of (steering_angle, throttle, navigation_info)
#         """
#         # Calculate time delta
#         if dt is None:
#             current_time = time.time()
#             dt = current_time - self.last_update_time
#             self.last_update_time = current_time
#         else:
#             self.last_update_time = time.time()
        
#         if dt <= 0:
#             dt = 0.01  # Minimum time step
        
#         # STEP 1: Get lane planner output FIRST (to get throttle)
#         angle, throttle = self.lane_planner.run(
#             lane_offset, lane_heading, detections
#         )
        
#         # STEP 2: Update speed sensor with new throttle
#         if isinstance(self.speed_sensor, ThrottleBasedSpeedSensor):
#             self.speed_sensor.update_throttle(throttle)
        
#         # STEP 3: Update navigation position with CURRENT speed
#         if self.is_navigating and self.nav_system.car_position:
#             if self.odometry_sensor:
#                 # Use odometry for more accurate position
#                 distance = self.odometry_sensor.get_distance_traveled()
#                 if distance > 0:
#                     # Update position based on distance traveled
#                     self._update_position_from_distance(distance)
#             else:
#                 # Use speed-based update
#                 current_speed = self.speed_sensor.get_speed()
#                 self.nav_system.update_position(current_speed, dt)
            
#             # Check if reached destination
#             if self.nav_system.get_remaining_distance() < 0.1:
#                 self.is_navigating = False
#                 print("[DONE] Reached destination!")
        
#         # STEP 4: Get navigation information
#         nav_info = self._get_navigation_info()
        
#         # STEP 5: Adjust steering based on navigation
#         angle = self._apply_navigation_steering(angle, nav_info)
        
#         self.last_throttle = throttle
        
#         return angle, throttle, nav_info
    
#     def _update_position_from_distance(self, distance: float):
#         """
#         Update position based on distance traveled (odometry)
#         More accurate than speed-based updates
#         """
#         if self.nav_system.car_position is None or self.nav_system.current_path is None:
#             return
        
#         current_idx = self.nav_system._get_current_path_index()
#         if current_idx is None or current_idx >= len(self.nav_system.current_path) - 1:
#             return
        
#         current_node = self.nav_system.current_path[current_idx]
#         next_node = self.nav_system.current_path[current_idx + 1]
#         edge_length = self.nav_system.graph[current_node].get(next_node, 0)
        
#         if edge_length == 0:
#             return
        
#         # Calculate remaining distance on current edge
#         remaining_on_edge = (1.0 - self.nav_system.car_position.progress) * edge_length
        
#         if distance >= remaining_on_edge:
#             # Move to next node
#             distance -= remaining_on_edge
#             self.nav_system.car_position.node = next_node
#             self.nav_system.car_position.progress = 0.0
            
#             # Continue to next edge if distance remains
#             if distance > 0 and current_idx + 1 < len(self.nav_system.current_path) - 1:
#                 self._update_position_from_distance(distance)
#         else:
#             # Still on current edge
#             self.nav_system.car_position.progress += distance / edge_length
    
#     def _apply_navigation_steering(self, base_angle: float, nav_info: Dict) -> float:
#         """
#         Adjust steering angle based on navigation requirements
#         """
#         angle = base_angle
        
#         junction = nav_info.get("junction_decision")
#         if junction:
#             # Close to junction - navigation takes priority
#             if junction.distance_to_junction < 0.3:
#                 if junction.direction == "left":
#                     angle = max(angle, -0.8)  # Ensure left turn
#                 elif junction.direction == "right":
#                     angle = min(angle, 0.8)   # Ensure right turn
#                 elif junction.direction == "straight":
#                     # Keep angle close to center
#                     if abs(angle) > 0.3:
#                         angle = angle * 0.5
        
#         return angle
    
#     def _get_navigation_info(self) -> Dict:
#         """Get current navigation status information"""
#         info = {
#             "is_navigating": self.is_navigating,
#             "car_position": str(self.nav_system.car_position) if self.nav_system.car_position else None,
#             "destination": self.nav_system.rider_position,
#             "remaining_distance": self.nav_system.get_remaining_distance(),
#             "next_waypoint": self.nav_system.get_next_waypoint(),
#             "junction_decision": self.nav_system.get_junction_decision(),
#             "current_path": self.nav_system.current_path,
#             "current_speed": self.speed_sensor.get_speed() if self.speed_sensor else 0.0,
#         }
#         return info
    
#     def get_junction_instruction(self) -> Optional[str]:
#         """Get current junction instruction for display"""
#         junction = self.nav_system.get_junction_decision()
#         if junction:
#             return junction.instruction
#         return None
    
#     def stop_navigation(self):
#         """Stop current navigation"""
#         self.is_navigating = False
#         print("Navigation stopped")


# # Example: Custom speed sensor implementation
# class EncoderSpeedSensor(SpeedSensor):
#     """
#     Example implementation using wheel encoder
#     """
    
#     def __init__(self, encoder, wheel_circumference: float):
#         """
#         Args:
#             encoder: Your encoder object (must have get_rpm() method)
#             wheel_circumference: Wheel circumference in meters
#         """
#         self.encoder = encoder
#         self.wheel_circumference = wheel_circumference
        
#     def get_speed(self) -> float:
#         """Calculate speed from encoder RPM"""
#         rpm = self.encoder.get_rpm()
#         # Convert RPM to m/s: (rpm / 60) * circumference
#         speed_mps = (rpm / 60.0) * self.wheel_circumference
#         # Convert to graph units (adjust conversion factor as needed)
#         return speed_mps * 1.0  # Adjust multiplier to match your graph units


# # Example: Custom odometry sensor
# class EncoderOdometry(OdometrySensor):
#     """
#     Example implementation using wheel encoder for distance
#     """
    
#     def __init__(self, encoder, wheel_circumference: float):
#         self.encoder = encoder
#         self.wheel_circumference = wheel_circumference
#         self.last_ticks = 0
#         self.ticks_per_revolution = 360  # Adjust to your encoder
        
#     def get_distance_traveled(self) -> float:
#         """Get distance since last call"""
#         current_ticks = self.encoder.get_ticks()
#         delta_ticks = current_ticks - self.last_ticks
#         self.last_ticks = current_ticks
        
#         # Convert ticks to distance
#         revolutions = delta_ticks / self.ticks_per_revolution
#         distance = revolutions * self.wheel_circumference
        
#         # Convert to graph units
#         return distance * 1.0  # Adjust multiplier to match your graph units
    
#     def reset(self):
#         """Reset odometry"""
#         self.last_ticks = self.encoder.get_ticks()


# if __name__ == "__main__":
#     print("=== Improved Navigation Synchronization Demo ===\n")
    
#     # Create controller with throttle-based speed (fallback)
#     controller = ImprovedNavigationController(
#         throttle_to_speed_factor=2.0
#     )
    
#     # Start navigation
#     controller.start_navigation("A", "D")
    
#     print("\nSimulating improved synchronization...\n")
#     print("Key improvements:")
#     print("  1. Throttle calculated FIRST")
#     print("  2. Speed updated with CURRENT throttle")
#     print("  3. Position updated with CURRENT speed")
#     print("  4. No one-step delay!\n")
    
#     # Simulate updates
#     for i in range(20):
#         lane_offset = 0.0
#         lane_heading = 0.0
#         detections = None
        
#         angle, throttle, nav_info = controller.update(
#             lane_offset, lane_heading, detections, dt=0.1
#         )
        
#         print(f"Step {i+1}:")
#         print(f"  Throttle: {throttle:.2f} -> Speed: {nav_info['current_speed']:.2f} units/s")
#         print(f"  Position: {nav_info['car_position']}")
#         print(f"  Remaining: {nav_info['remaining_distance']:.2f} units")
        
#         if nav_info['junction_decision']:
#             print(f"  [JUNCTION] {nav_info['junction_decision'].instruction}")
        
#         print()
#         time.sleep(0.1)


