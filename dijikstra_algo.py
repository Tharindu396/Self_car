import heapq
import json
import os
import cv2
import pytesseract

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

# ----------------------------------------
# 1. LOAD GRAPH.JSON
# ----------------------------------------
_here = os.path.dirname(__file__)
_graph_path = os.path.join(_here, "graph.json")

with open(_graph_path, "r", encoding="utf-8") as _f:
    graph = json.load(_f)

# ----------------------------------------
# 2. DIJKSTRA SHORTEST PATH FUNCTION
# ----------------------------------------
def dijkstra(graph, start, goal):
    """
    Find shortest path using Dijkstra's algorithm
    
    Args:
        graph: Road graph dictionary
        start: Starting node
        goal: Destination node
        
    Returns:
        Tuple of (path as list of nodes, total cost)
    """
    if start not in graph:
        raise ValueError(f"Start node '{start}' not in graph")
    if goal not in graph:
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

        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path))

    return None, float("inf")

# ----------------------------------------
# 3. OCR FUNCTION TO DETECT START NODE
# ----------------------------------------
def detect_start_node_from_camera():
    """
    Use OCR to detect start node from camera
    
    Returns:
        Detected node string or None
    """
    cam = cv2.VideoCapture(0)
    
    if not cam.isOpened():
        print("Warning: Webcam not detected. Using manual input.")
        return None
    
    print("Webcam opened. Press ESC to stop, SPACE to capture.")
    
    text = ""
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert BGR to RGB for OCR
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Extract text from the frame
        try:
            text = pytesseract.image_to_string(rgb_frame)
            text = text.strip().upper()
        except Exception as e:
            print(f"OCR Error: {e}")
            break

        # Show webcam feed with detected text
        cv2.putText(frame, f"Detected: {text}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Live OCR - Press ESC to Exit, SPACE to Capture", frame)

        # Exit on ESC, capture on SPACE
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            text = None
            break
        elif key == 32:  # SPACE - capture current text
            break
    
    cam.release()
    cv2.destroyAllWindows()
    
    return text

# ----------------------------------------
# 4. MAIN FUNCTION
# ----------------------------------------
if __name__ == "__main__":
    print("=== Dijkstra Pathfinding with OCR ===\n")
    
    # Try to detect start node from camera
    start = detect_start_node_from_camera()
    
    # If OCR failed or user wants manual input
    if start is None or start not in graph:
        print("\nEnter nodes manually:")
        start = input("Enter start node: ").strip().upper()
    
    goal = input("Enter goal node: ").strip().upper()
    
    # Find path
    path, distance = dijkstra(graph, start, goal)
    
    if path:
        print(f"\nShortest Path: {' -> '.join(path)}")
        print(f"Total Distance: {distance:.2f} units")
    else:
        print(f"\nNo path found from {start} to {goal}")
