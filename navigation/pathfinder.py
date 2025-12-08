"""
Pathfinding Module

Implements Dijkstra's shortest path algorithm for navigation planning.
"""

from typing import Dict, List, Optional, Tuple
import heapq
from graph_loader import Graph


class PathFinder:
    """
    Dijkstra-based pathfinding for navigation graph.
    """
    
    def __init__(self, graph: Graph):
        """
        Initialize pathfinder with a graph.
        
        Args:
            graph: Graph instance with adjacency and node coordinates
        """
        self.graph = graph

    def find_shortest_path(self, start: str, goal: str) -> Optional[List[str]]:
        """
        Find shortest path from start to goal using Dijkstra's algorithm.
        
        Args:
            start: Starting node
            goal: Goal node
            
        Returns:
            List of nodes representing path, or None if no path exists
        """
        if start not in self.graph.nodes() or goal not in self.graph.nodes():
            return None
        
        # Dijkstra's algorithm
        distances = {node: float('inf') for node in self.graph.nodes()}
        distances[start] = 0
        previous = {node: None for node in self.graph.nodes()}
        
        # Priority queue: (distance, node)
        pq = [(0, start)]
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            # Early termination if we reached the goal
            if current == goal:
                break
            
            # Check neighbors
            for neighbor, edge_cost in self.graph.neighbors(current).items():
                if neighbor not in visited:
                    new_dist = current_dist + edge_cost
                    
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        previous[neighbor] = current
                        heapq.heappush(pq, (new_dist, neighbor))
        
        # Reconstruct path
        if distances[goal] == float('inf'):
            return None  # No path found
        
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous[current]
        
        path.reverse()
        return path

    def get_path_instructions(self, path: List[str]) -> List[Dict]:
        """
        Convert path to detailed instructions with turn directions.
        
        Args:
            path: List of nodes from start to goal
            
        Returns:
            List of instruction dictionaries with:
                - 'node': current node
                - 'action': 'start', 'move', 'turn'
                - 'direction': 'left', 'right', 'straight'
                - 'next_node': next node in path
                - 'distance': estimated distance
        """
        if not path or len(path) < 2:
            return []
        
        instructions = []
        
        for i, node in enumerate(path):
            next_node = path[i + 1] if i + 1 < len(path) else None
            
            if i == 0:
                # Starting node
                instruction = {
                    'node': node,
                    'action': 'start',
                    'direction': 'forward',
                    'next_node': next_node,
                    'distance': self.graph.get_distance_between(node, next_node)
                }
            else:
                # Determine turn direction
                prev_node = path[i - 1]
                turn_direction = self.graph.get_turn_action(prev_node, node, next_node) if next_node else 'straight'
                
                instruction = {
                    'node': node,
                    'action': 'turn' if turn_direction != 'straight' else 'move',
                    'direction': turn_direction,
                    'next_node': next_node,
                    'distance': self.graph.get_distance_between(node, next_node) if next_node else 0
                }
            
            instructions.append(instruction)
        
        return instructions

    def get_total_distance(self, path: List[str]) -> float:
        """
        Calculate total distance for a path.
        
        Args:
            path: List of nodes
            
        Returns:
            Total distance in graph units
        """
        total = 0.0
        for i in range(len(path) - 1):
            dist = self.graph.get_distance_between(path[i], path[i + 1])
            if dist is not None:
                total += dist
        return total

    def find_alternative_paths(self, start: str, goal: str, num_paths: int = 3) -> List[List[str]]:
        """
        Find multiple alternative paths (using Yen's k-shortest paths algorithm concept).
        
        Args:
            start: Starting node
            goal: Goal node
            num_paths: Number of alternative paths to find
            
        Returns:
            List of paths sorted by distance
        """
        paths = []
        
        # Simple approach: find shortest path, then progressively penalize edges
        temp_graph_data = self.graph.adj.copy()
        
        for _ in range(num_paths):
            path = self.find_shortest_path(start, goal)
            
            if not path or path in paths:
                break
            
            paths.append(path)
            
            # Penalize edges in this path for next iteration
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                if u in self.graph.adj and v in self.graph.adj[u]:
                    self.graph.adj[u][v] *= 1.5  # Increase cost
        
        # Restore original graph
        self.graph.adj = temp_graph_data
        
        return paths
