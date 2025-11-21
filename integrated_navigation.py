# """
# Integration module connecting navigation system with lane change planner
# Provides unified interface for RPI car navigation
# """

# from typing import Dict, List, Optional, Tuple
# from navigation_system import NavigationSystem, Position, JunctionDecision
# import time

# # Note: planner_part dependency removed - use standalone_navigation.py instead
# # This file is kept for backward compatibility but may not work without planner_part


# class IntegratedNavigationController:
#     """
#     Integrates pathfinding navigation with lane change planning
#     Combines high-level route planning with low-level lane control
#     """
    
#     def __init__(
#         self,
#         graph_path: str = "graph.json",
#         max_throttle: float = 0.25,
#         safety_gap_m: float = 1.2,
#     ):
#         """Initialize integrated navigation controller"""
#         self.nav_system = NavigationSystem(graph_path)
#         self.lane_planner = LaneChangePlanner(
#             max_throttle=max_throttle,
#             safety_gap_m=safety_gap_m
#         )
        
#         self.current_speed: float = 0.0
#         self.is_navigating: bool = False
        
#     def start_navigation(self, start_node: str, destination_node: str) -> bool:
#         """
#         Start navigation to destination
        
#         Args:
#             start_node: Starting node
#             destination_node: Destination node
            
#         Returns:
#             True if route found and navigation started
#         """
#         success = self.nav_system.set_route(start_node, destination_node)
#         if success:
#             self.is_navigating = True
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
#         Main update function - combines navigation and lane planning
        
#         IMPROVED SYNCHRONIZATION:
#         - Calculates throttle FIRST
#         - Updates speed with CURRENT throttle
#         - Updates position with CURRENT speed
#         - Eliminates one-step delay
        
#         Args:
#             lane_offset: Current lane offset
#             lane_heading: Current lane heading
#             detections: Object detections
#             dt: Time delta (auto if None)
            
#         Returns:
#             Tuple of (steering_angle, throttle, navigation_info)
#         """
#         # STEP 1: Get lane planner output FIRST (to get throttle)
#         angle, throttle = self.lane_planner.run(
#             lane_offset, lane_heading, detections
#         )
        
#         # STEP 2: Calculate speed from CURRENT throttle (not previous)
#         self.current_speed = throttle * 2.0  # Convert throttle to speed
        
#         # STEP 3: Update navigation position with CURRENT speed
#         if self.is_navigating and self.nav_system.car_position:
#             # Use current speed (from current throttle) instead of old speed
#             self.nav_system.update_position(self.current_speed, dt)
            
#             # Check if reached destination
#             if self.nav_system.get_remaining_distance() < 0.1:
#                 self.is_navigating = False
#                 print("[DONE] Reached destination!")
        
#         # STEP 4: Get navigation information
#         nav_info = self._get_navigation_info()
        
#         # STEP 5: Adjust steering based on navigation if needed
#         if nav_info.get("junction_decision"):
#             # At junction, navigation takes priority
#             junction = nav_info["junction_decision"]
#             if junction.distance_to_junction < 0.3:
#                 # Close to junction - adjust angle based on direction
#                 if junction.direction == "left":
#                     angle = max(angle, -0.8)  # Ensure left turn
#                 elif junction.direction == "right":
#                     angle = min(angle, 0.8)   # Ensure right turn
        
#         return angle, throttle, nav_info
    
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


# # Example usage with DonkeyCar integration
# def create_donkeycar_integration():
#     """
#     Example function showing how to integrate with DonkeyCar vehicle
#     This can be added to manage.py
#     """
#     integration_code = '''
# from integrated_navigation import IntegratedNavigationController

# # Add to your Vehicle setup:
# nav_controller = IntegratedNavigationController()
# V.add(nav_controller, 
#       inputs=['lane/offset', 'lane/heading', 'det/objs'],
#       outputs=['angle', 'throttle', 'nav/info'])

# # To start navigation:
# # nav_controller.start_navigation("A", "D")
# '''
#     return integration_code


# if __name__ == "__main__":
#     # Demo
#     print("=== Integrated Navigation Controller Demo ===\n")
    
#     controller = IntegratedNavigationController()
    
#     # Start navigation
#     controller.start_navigation("A", "D")
    
#     # Simulate updates
#     print("\nSimulating navigation updates...\n")
#     for i in range(20):
#         # Simulate lane detection inputs
#         lane_offset = 0.0
#         lane_heading = 0.0
#         detections = None
        
#         angle, throttle, nav_info = controller.update(
#             lane_offset, lane_heading, detections, dt=0.1
#         )
        
#         print(f"Step {i+1}:")
#         print(f"  Steering: {angle:.2f}, Throttle: {throttle:.2f}")
#         print(f"  Position: {nav_info['car_position']}")
#         print(f"  Remaining: {nav_info['remaining_distance']:.2f} units")
        
#         if nav_info['junction_decision']:
#             print(f"  [JUNCTION] {nav_info['junction_decision'].instruction}")
        
#         print()
#         time.sleep(0.1)

