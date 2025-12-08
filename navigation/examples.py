"""
Navigation System Usage Examples

Demonstrates how to use the navigation system for:
1. Graph loading and pathfinding
2. Junction detection from camera frames
3. Real-time navigation control
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from navigation import NavigationController, Graph, PathFinder
from navigation.junction_detector import JunctionDetector


def example_1_basic_pathfinding():
    """
    Example 1: Load graph and find path from A to D
    """
    print("\n" + "="*60)
    print("EXAMPLE 1: Basic Pathfinding")
    print("="*60)
    
    # Load graph from JSON
    graph_path = "../graph.json"  # Adjust path as needed
    graph = Graph.from_json(graph_path)
    
    print(f"\n[INFO] Graph loaded with {len(graph.nodes())} nodes")
    print(f"[INFO] Nodes: {', '.join(graph.nodes())}")
    
    # Create pathfinder
    pathfinder = PathFinder(graph)
    
    # Find path from A to D
    start, goal = "A", "D"
    path = pathfinder.find_shortest_path(start, goal)
    
    if path:
        print(f"\n[SUCCESS] Path found from {start} to {goal}:")
        print(f"  {' -> '.join(path)}")
        
        # Get detailed instructions
        instructions = pathfinder.get_path_instructions(path)
        print(f"\n[INFO] Instructions:")
        for i, instr in enumerate(instructions):
            print(f"  Step {i+1}: {instr['node']} - {instr['action']} {instr['direction']}", end="")
            if instr.get('distance'):
                print(f" (distance: {instr['distance']:.2f})", end="")
            print()
        
        # Total distance
        total_dist = pathfinder.get_total_distance(path)
        print(f"\n[INFO] Total distance: {total_dist:.2f} units")
    else:
        print(f"[ERROR] No path found from {start} to {goal}")


def example_2_turn_detection():
    """
    Example 2: Detect turn directions at junctions
    """
    print("\n" + "="*60)
    print("EXAMPLE 2: Turn Direction Detection")
    print("="*60)
    
    graph_path = "../graph.json"
    graph = Graph.from_json(graph_path)
    pathfinder = PathFinder(graph)
    
    # Find a path with multiple turns
    path = pathfinder.find_shortest_path("A", "C")
    
    if path and len(path) >= 3:
        print(f"\n[INFO] Path: {' -> '.join(path)}")
        print(f"\n[INFO] Turn directions:")
        
        for i in range(1, len(path) - 1):
            prev = path[i-1]
            curr = path[i]
            next_node = path[i+1]
            
            turn = graph.get_turn_action(prev, curr, next_node)
            heading = graph.get_heading_to_node(curr, next_node)
            distance = graph.get_distance_between(curr, next_node)
            
            print(f"  At {curr}: {turn.upper()} turn", end="")
            if heading:
                print(f" (heading: {heading:.1f}°)", end="")
            if distance:
                print(f" [distance to {next_node}: {distance:.2f}]", end="")
            print()


def example_3_navigation_controller():
    """
    Example 3: Use integrated NavigationController
    """
    print("\n" + "="*60)
    print("EXAMPLE 3: Navigation Controller")
    print("="*60)
    
    graph_path = "../graph.json"
    
    # Initialize controller
    nav = NavigationController(
        graph_path=graph_path,
        lane_width_pixels=80,
        junction_threshold=0.7
    )
    
    print("\n[INFO] Navigation controller initialized")
    
    # Start navigation
    start, goal = "A", "E"
    if nav.start_navigation(start, goal):
        print(f"\n[SUCCESS] Navigation started: {start} -> {goal}")
        
        # Get status
        status = nav.get_current_status()
        print(f"\n[STATUS]")
        print(f"  State: {status['state']}")
        print(f"  Current node: {status['current_node']}")
        print(f"  Goal node: {status['goal_node']}")
        print(f"  Path progress: {status['path_progress']}")
        print(f"  Remaining nodes: {status['remaining_nodes']}")
        
        # Simulate position updates
        print(f"\n[INFO] Simulating navigation...")
        for node in status['path'][:3]:  # Simulate first 3 nodes
            nav.update_position(node)
            status = nav.get_current_status()
            print(f"  Updated at {node}: progress {status['path_progress']}")
        
        # Save path
        nav.save_current_path("current_path.json")
    else:
        print(f"[ERROR] Failed to start navigation")


def example_4_junction_detection_metrics():
    """
    Example 4: Understanding junction detection metrics
    """
    print("\n" + "="*60)
    print("EXAMPLE 4: Junction Detection Metrics")
    print("="*60)
    
    detector = JunctionDetector(
        lane_width_pixels=80,
        junction_sensitivity=0.6,
        min_junction_size=50
    )
    
    print("\n[INFO] Junction Detector initialized with:")
    print(f"  Lane width: {detector.lane_width} pixels")
    print(f"  Sensitivity: {detector.sensitivity}")
    print(f"  Min junction size: {detector.min_size} pixels")
    
    print("\n[INFO] Junction detection works by:")
    print("  1. Converting frame to grayscale")
    print("  2. Detecting edges using Canny detector")
    print("  3. Detecting lines using Hough transform")
    print("  4. Analyzing line patterns to classify junction type")
    print("  5. Applying temporal filtering to reduce false positives")
    
    print("\n[INFO] Possible junction types:")
    print("  - 'straight': No junction, continue forward")
    print("  - 'left': Left turn detected")
    print("  - 'right': Right turn detected")
    print("  - 'intersection': T-junction or crossroads")
    print("  - 'cross': Four-way intersection")


def example_5_alternative_paths():
    """
    Example 5: Find alternative paths
    """
    print("\n" + "="*60)
    print("EXAMPLE 5: Alternative Path Planning")
    print("="*60)
    
    graph_path = "../graph.json"
    graph = Graph.from_json(graph_path)
    pathfinder = PathFinder(graph)
    
    start, goal = "A", "D"
    paths = pathfinder.find_alternative_paths(start, goal, num_paths=3)
    
    print(f"\n[INFO] Found {len(paths)} alternative paths from {start} to {goal}:")
    
    for i, path in enumerate(paths, 1):
        distance = pathfinder.get_total_distance(path)
        print(f"\n  Path {i}:")
        print(f"    Route: {' -> '.join(path)}")
        print(f"    Distance: {distance:.2f} units")
        print(f"    Nodes: {len(path)}")


def main():
    """Run all examples"""
    print("\n" + "#"*60)
    print("# NAVIGATION SYSTEM - USAGE EXAMPLES")
    print("#"*60)
    
    try:
        example_1_basic_pathfinding()
    except Exception as e:
        print(f"[ERROR] Example 1 failed: {e}")
    
    try:
        example_2_turn_detection()
    except Exception as e:
        print(f"[ERROR] Example 2 failed: {e}")
    
    try:
        example_3_navigation_controller()
    except Exception as e:
        print(f"[ERROR] Example 3 failed: {e}")
    
    try:
        example_4_junction_detection_metrics()
    except Exception as e:
        print(f"[ERROR] Example 4 failed: {e}")
    
    try:
        example_5_alternative_paths()
    except Exception as e:
        print(f"[ERROR] Example 5 failed: {e}")
    
    print("\n" + "#"*60)
    print("# END OF EXAMPLES")
    print("#"*60 + "\n")


if __name__ == "__main__":
    main()
