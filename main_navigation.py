"""
Main Navigation System - Integrates dijkstra_algo.py with navigation
Works standalone without planner_part, lane_det, or manage.py
"""

from dijikstra_algo import dijkstra, graph, detect_start_node_from_camera
from standalone_navigation import StandaloneNavigation, NavigationVisualizer
import time


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
    print("\nStep 4: Choose display mode")
    print("  1. Text-only (console output)")
    print("  2. Visualization (Uber-style map)")
    mode = input("Enter choice (1 or 2): ").strip()
    
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

