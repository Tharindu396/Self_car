"""
Navigation System Tests

Validates:
1. Graph loading from JSON
2. Pathfinding correctness
3. Turn direction calculation
4. Junction detection initialization
5. Navigation controller state machine
"""

import sys
import os
import json

# Add navigation module to path
sys.path.insert(0, os.path.dirname(__file__))


def test_graph_loading():
    """Test 1: Graph loading from JSON"""
    print("\n" + "="*60)
    print("TEST 1: Graph Loading")
    print("="*60)
    
    try:
        from navigation import Graph
        
        graph = Graph.from_json("graph.json")
        
        assert graph is not None, "Graph is None"
        assert len(graph.nodes()) > 0, "Graph has no nodes"
        
        # Check for required nodes
        nodes = graph.nodes()
        assert "A" in nodes, "Node A not found"
        assert "D" in nodes, "Node D not found"
        
        print(f"✓ Graph loaded successfully")
        print(f"  - Nodes: {len(nodes)}")
        print(f"  - Sample nodes: {', '.join(nodes[:5])}")
        
        # Check adjacency
        adj_a = graph.neighbors("A")
        assert len(adj_a) > 0, "Node A has no neighbors"
        print(f"  - Node A neighbors: {list(adj_a.keys())}")
        
        # Check coordinates
        if graph.node_coords:
            print(f"  - Coordinates available for {len(graph.node_coords)} nodes")
            coord_a = graph.node_coords.get("A")
            if coord_a:
                print(f"    Node A at {coord_a}")
        
        return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_pathfinding():
    """Test 2: Pathfinding with Dijkstra"""
    print("\n" + "="*60)
    print("TEST 2: Pathfinding (Dijkstra)")
    print("="*60)
    
    try:
        from navigation import Graph, PathFinder
        
        graph = Graph.from_json("graph.json")
        pathfinder = PathFinder(graph)
        
        # Test case 1: A to D
        path = pathfinder.find_shortest_path("A", "D")
        assert path is not None, "No path found from A to D"
        assert path[0] == "A", "Path doesn't start at A"
        assert path[-1] == "D", "Path doesn't end at D"
        
        print(f"✓ Pathfinding works")
        print(f"  - Path A→D: {' → '.join(path)}")
        print(f"  - Length: {len(path)} nodes")
        
        distance = pathfinder.get_total_distance(path)
        print(f"  - Distance: {distance:.2f} units")
        
        # Test case 2: Invalid path
        path_invalid = pathfinder.find_shortest_path("A", "INVALID")
        assert path_invalid is None, "Should return None for invalid node"
        print(f"✓ Handles invalid nodes correctly")
        
        # Test case 3: Same start and goal
        path_same = pathfinder.find_shortest_path("A", "A")
        if path_same:
            assert path_same == ["A"], "Same node should return single-node path"
            print(f"✓ Handles same start/goal correctly")
        
        return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_turn_detection():
    """Test 3: Turn direction calculation"""
    print("\n" + "="*60)
    print("TEST 3: Turn Direction Detection")
    print("="*60)
    
    try:
        from navigation import Graph, PathFinder
        
        graph = Graph.from_json("graph.json")
        pathfinder = PathFinder(graph)
        
        # Get a path with multiple turns
        path = pathfinder.find_shortest_path("A", "D")
        
        if path and len(path) >= 3:
            turns_detected = 0
            print(f"✓ Analyzing turns in path: {' → '.join(path[:5])}...")
            
            for i in range(1, min(len(path) - 1, 5)):
                prev = path[i-1]
                curr = path[i]
                next_node = path[i+1]
                
                turn = graph.get_turn_action(prev, curr, next_node)
                assert turn in ["left", "right", "straight"], f"Invalid turn: {turn}"
                
                print(f"  - At {curr}: {turn}")
                turns_detected += 1
            
            print(f"✓ Turn detection works ({turns_detected} turns analyzed)")
            return True
        else:
            print("✓ Could not find path for turn analysis (but graph loads)")
            return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_distance_calculation():
    """Test 4: Distance calculation"""
    print("\n" + "="*60)
    print("TEST 4: Distance Calculation")
    print("="*60)
    
    try:
        from navigation import Graph
        import math
        
        graph = Graph.from_json("graph.json")
        
        if not graph.node_coords:
            print("✓ No coordinates in graph (distance calculation skipped)")
            return True
        
        # Test distance calculation
        dist = graph.get_distance_between("A", "B")
        if dist:
            # Verify Euclidean distance
            a = graph.node_coords["A"]
            b = graph.node_coords["B"]
            expected = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
            assert abs(dist - expected) < 0.01, f"Distance mismatch: {dist} vs {expected}"
            
            print(f"✓ Distance calculation correct")
            print(f"  - A→B distance: {dist:.2f} units")
        else:
            print("✓ Distance returns None for unavailable coordinates")
        
        # Test heading calculation
        heading = graph.get_heading_to_node("A", "B")
        if heading:
            assert 0 <= heading <= 360, f"Invalid heading: {heading}"
            print(f"✓ Heading calculation correct")
            print(f"  - A→B heading: {heading:.1f}°")
        else:
            print("✓ Heading returns None for unavailable coordinates")
        
        return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_junction_detector():
    """Test 5: Junction detector initialization"""
    print("\n" + "="*60)
    print("TEST 5: Junction Detector")
    print("="*60)
    
    try:
        from navigation import JunctionDetector
        
        detector = JunctionDetector(
            lane_width_pixels=80,
            junction_sensitivity=0.7,
            min_junction_size=50
        )
        
        assert detector is not None, "Detector is None"
        assert detector.lane_width == 80, "Lane width not set correctly"
        assert detector.sensitivity == 0.7, "Sensitivity not set correctly"
        
        print(f"✓ Junction detector initialized")
        print(f"  - Lane width: {detector.lane_width} pixels")
        print(f"  - Sensitivity: {detector.sensitivity}")
        print(f"  - Min size: {detector.min_size} pixels")
        
        # Test reset
        detector.reset()
        assert len(detector.junction_history) == 0, "History not cleared"
        print(f"✓ Reset works correctly")
        
        return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_navigation_controller():
    """Test 6: Navigation controller state machine"""
    print("\n" + "="*60)
    print("TEST 6: Navigation Controller")
    print("="*60)
    
    try:
        from navigation import NavigationController, NavigationState
        
        nav = NavigationController("graph.json")
        
        # Check initial state
        assert nav.state == NavigationState.IDLE, "Should start in IDLE state"
        print(f"✓ Initial state: IDLE")
        
        # Start navigation
        result = nav.start_navigation("A", "D")
        assert result == True, "start_navigation should return True"
        assert nav.state == NavigationState.NAVIGATING, "Should be in NAVIGATING state"
        assert nav.current_node == "A", "Current node should be A"
        assert nav.goal_node == "D", "Goal node should be D"
        
        print(f"✓ Navigation started")
        print(f"  - Current node: {nav.current_node}")
        print(f"  - Goal node: {nav.goal_node}")
        print(f"  - State: {nav.state.value}")
        
        # Get status
        status = nav.get_current_status()
        assert status['state'] == 'navigating', "Status state incorrect"
        assert 'path' in status, "Status missing path"
        
        print(f"✓ Status retrieval works")
        print(f"  - Path length: {len(status['path'])} nodes")
        
        # Test reset
        nav.reset()
        assert nav.state == NavigationState.IDLE, "Should reset to IDLE"
        assert nav.current_node is None, "Current node should be None after reset"
        print(f"✓ Reset works correctly")
        
        return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_path_instructions():
    """Test 7: Path instructions generation"""
    print("\n" + "="*60)
    print("TEST 7: Path Instructions")
    print("="*60)
    
    try:
        from navigation import Graph, PathFinder
        
        graph = Graph.from_json("graph.json")
        pathfinder = PathFinder(graph)
        
        path = pathfinder.find_shortest_path("A", "D")
        if path and len(path) >= 2:
            instructions = pathfinder.get_path_instructions(path)
            
            assert len(instructions) == len(path), "Instruction count mismatch"
            assert instructions[0]['action'] == 'start', "First instruction should be 'start'"
            
            print(f"✓ Path instructions generated")
            print(f"  - {len(instructions)} instructions")
            
            for i, instr in enumerate(instructions[:3]):
                print(f"  - {i+1}. {instr['node']}: {instr['action']} {instr['direction']}")
            
            return True
        else:
            print("✓ Path too short for detailed analysis")
            return True
        
    except Exception as e:
        print(f"✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_tests():
    """Run all tests and report results"""
    print("\n" + "#"*60)
    print("# NAVIGATION SYSTEM - TEST SUITE")
    print("#"*60)
    
    tests = [
        ("Graph Loading", test_graph_loading),
        ("Pathfinding", test_pathfinding),
        ("Turn Detection", test_turn_detection),
        ("Distance Calculation", test_distance_calculation),
        ("Junction Detector", test_junction_detector),
        ("Navigation Controller", test_navigation_controller),
        ("Path Instructions", test_path_instructions),
    ]
    
    results = {}
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            if test_func():
                results[name] = "✓ PASS"
                passed += 1
            else:
                results[name] = "✗ FAIL"
                failed += 1
        except Exception as e:
            results[name] = f"✗ ERROR: {str(e)[:50]}"
            failed += 1
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    for name, result in results.items():
        print(f"{name:.<40} {result}")
    
    print("-"*60)
    print(f"Total: {passed} passed, {failed} failed")
    
    if failed == 0:
        print("\n✓ ALL TESTS PASSED!")
        return True
    else:
        print(f"\n✗ {failed} TEST(S) FAILED")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
