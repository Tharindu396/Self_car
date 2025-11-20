import heapq
import json
import os
import cv2
import pytesseract

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

# ----------------------------------------
# 2. LOAD GRAPH.JSON
# ----------------------------------------
_here = os.path.dirname(__file__)
_graph_path = os.path.join(_here, "graph.json")

with open(_graph_path, "r", encoding="utf-8") as _f:
    graph = json.load(_f)

# ----------------------------------------
# 3. SETUP WEBCAM
# ----------------------------------------
cam = cv2.VideoCapture(0)

if not cam.isOpened():
    raise Exception("Error: Webcam not detected!")

print("Webcam opened. Press ESC to stop.")

text = ""
# ----------------------------------------
# 4. LIVE OCR LOOP
# ----------------------------------------
while True:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert BGR → RGB for OCR
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Extract text from the frame
    try:
        text = pytesseract.image_to_string(rgb_frame)
    except Exception as e:
        print("OCR Error:", e)
        break

    # Show webcam feed
    cv2.imshow("Live OCR - Press ESC to Exit", frame)

    # Print extracted text
    print("\n--- Extracted Text From Frame ---")
    print(text.strip())

    # Exit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break

cam.release()
cv2.destroyAllWindows()

# ----------------------------------------
# 5. DIJKSTRA SHORTEST PATH FUNCTION
# ----------------------------------------
def dijkstra(graph, start, goal):
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
            heapq.heappush(pq, (cost + weight, neighbor, path))

    return None, float("inf")

# ----------------------------------------
# 6. USER INPUT FOR PATHFINDING
# ----------------------------------------
if __name__ == "__main__":
    start = input("Enter start node: ")
    goal = input("Enter goal node: ")
    print(text)
    path, distance = dijkstra(graph, text, goal)

    print("\nShortest Path:", path)
    print("Distance:", distance)
