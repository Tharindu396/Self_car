import heapq
import json
import os

# load graph from a JSON file located next to this script
_here = os.path.dirname(__file__)
_graph_path = os.path.join(_here, "graph.json")
with open(_graph_path, "r", encoding="utf-8") as _f:
    graph = json.load(_f)

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

    return None

if __name__ == "__main__":
    start = input("Enter start node: ")
    goal = input("Enter goal node: ")
    path, distance = dijkstra(graph, start, goal)
    print("Shortest Path:", path)
    print("Distance:", distance)