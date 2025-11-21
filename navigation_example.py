# """
# Example usage of the navigation system
# Shows how to use real-time tracking, visualization, and junction decisions
# """

# from navigation_system import NavigationSystem, NavigationVisualizer
# import time
# import matplotlib.pyplot as plt


# def example_basic_navigation():
#     """Basic navigation example"""
#     print("=== Basic Navigation Example ===\n")
    
#     # Create navigation system
#     nav = NavigationSystem()
    
#     # Set a route
#     nav.set_route("A", "D")
    
#     # Simulate car movement
#     speed = 0.5  # units per second
#     print("\nSimulating car movement...\n")
    
#     for i in range(30):
#         nav.update_position(speed, dt=0.1)
        
#         print(f"Step {i+1}:")
#         print(f"  Position: {nav.car_position}")
#         print(f"  Remaining distance: {nav.get_remaining_distance():.2f} units")
        
#         # Check for junction decisions
#         junction = nav.get_junction_decision()
#         if junction:
#             print(f"  [JUNCTION] {junction.instruction}")
#             print(f"     Direction: {junction.direction}")
#             print(f"     Distance: {junction.distance_to_junction:.2f} units")
        
#         print()
#         time.sleep(0.1)
        
#         if nav.get_remaining_distance() < 0.1:
#             print("[DONE] Reached destination!")
#             break


# def example_with_visualization():
#     """Navigation with real-time visualization (Uber-style)"""
#     print("=== Navigation with Visualization ===\n")
    
#     # Create navigation system
#     nav = NavigationSystem()
    
#     # Set route
#     nav.set_route("A", "D")
    
#     # Create visualizer
#     try:
#         viz = NavigationVisualizer(nav)
#         print("Visualization window opened.")
#         print("The window shows:")
#         print("  - Blue line: Planned route (like Uber)")
#         print("  - Red triangle: Your car position")
#         print("  - Green circle: Rider/destination")
#         print("  - Orange circles: Junctions")
#         print("  - Yellow boxes: Junction instructions")
#         print("\nClose the window to stop.\n")
        
#         # Simulate movement
#         speed = 0.5
#         running = True
        
#         try:
#             while running:
#                 still_on_route = nav.update_position(speed, dt=0.1)
                
#                 # Get junction decision
#                 junction = nav.get_junction_decision()
#                 if junction:
#                     print(f"[JUNCTION] {junction.instruction}")
                
#                 # Update visualization
#                 viz.update_and_draw()
                
#                 if not still_on_route:
#                     print("\n[DONE] Reached destination!")
#                     # Keep showing final state for 2 seconds
#                     for _ in range(20):
#                         viz.update_and_draw()
#                         time.sleep(0.1)
#                     break
                
#                 time.sleep(0.1)
                
#         except KeyboardInterrupt:
#             print("\nStopped by user")
#         finally:
#             plt.close('all')
            
#     except Exception as e:
#         print(f"Visualization error: {e}")
#         print("Running without visualization...")
#         example_basic_navigation()


# def example_multiple_routes():
#     """Example with multiple route options"""
#     print("=== Multiple Routes Example ===\n")
    
#     nav = NavigationSystem()
    
#     routes = [
#         ("A", "D"),
#         ("B", "F"),
#         ("E", "C"),
#     ]
    
#     for start, dest in routes:
#         print(f"\n--- Route: {start} -> {dest} ---")
#         if nav.set_route(start, dest):
#             path = nav.current_path
#             distance = nav.path_cost
#             print(f"Path: {' -> '.join(path)}")
#             print(f"Total distance: {distance:.2f} units")
            
#             # Simulate quick movement
#             for _ in range(5):
#                 nav.update_position(1.0, dt=0.1)
#             print(f"After 0.5s: Position = {nav.car_position}")
#             print(f"Remaining: {nav.get_remaining_distance():.2f} units")


# def example_junction_decisions():
#     """Focus on junction decision making"""
#     print("=== Junction Decision Example ===\n")
    
#     nav = NavigationSystem()
    
#     # Route that goes through multiple junctions
#     nav.set_route("A", "D")
    
#     print("Following route through junctions...\n")
    
#     speed = 0.3  # Slower to see junction decisions clearly
#     junction_count = 0
    
#     for i in range(100):
#         nav.update_position(speed, dt=0.1)
        
#         junction = nav.get_junction_decision()
#         if junction:
#             junction_count += 1
#             print(f"[JUNCTION] Decision #{junction_count}:")
#             print(f"   Current: {junction.current_node}")
#             print(f"   Next: {junction.next_node}")
#             print(f"   Direction: {junction.direction.upper()}")
#             print(f"   Distance: {junction.distance_to_junction:.2f} units")
#             print(f"   Instruction: {junction.instruction}")
#             print()
        
#         if nav.get_remaining_distance() < 0.1:
#             break
        
#         time.sleep(0.05)


# if __name__ == "__main__":
#     import sys
    
#     if len(sys.argv) > 1:
#         mode = sys.argv[1]
#         if mode == "basic":
#             example_basic_navigation()
#         elif mode == "viz":
#             example_with_visualization()
#         elif mode == "routes":
#             example_multiple_routes()
#         elif mode == "junctions":
#             example_junction_decisions()
#         else:
#             print(f"Unknown mode: {mode}")
#             print("Available modes: basic, viz, routes, junctions")
#     else:
#         # Run visualization example by default
#         example_with_visualization()

