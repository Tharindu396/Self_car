"""
Graph Loader Module

Loads the navigation graph from JSON and provides utilities for:
- Pathfinding (via Dijkstra)
- Turn direction calculation
- Node management
"""

import json
import math
from typing import Dict, List, Tuple, Optional


class Graph:
    """
    Navigation graph with nodes and adjacency information.
    
    Attributes:
        adj: Adjacency dictionary { "A": {"B": cost, "C": cost}, ... }
        node_coords: Node coordinates { "A": [x, y], ... }
    """
    
    def __init__(self, adjacency: Dict[str, Dict[str, float]], nodes: Dict[str, List[float]] = None):
        """
        Initialize graph with adjacency and optional node coordinates.
        
        Args:
            adjacency: { "A": {"B": cost, "C": cost}, ... }
            nodes: { "A": [x, y], ... }
        """
        self.adj = adjacency
        self.node_coords = nodes if nodes else {}

    @classmethod
    def from_json(cls, path: str) -> "Graph":
        """
        Load graph from JSON file.
        Supports both new format (with "nodes" and "adjacency") and old format (adjacency only).
        
        Args:
            path: Path to JSON file
            
        Returns:
            Graph instance
        """
        with open(path, "r") as f:
            data = json.load(f)
        
        # Check if new format with "nodes" and "adjacency"
        if "nodes" in data and "adjacency" in data:
            coords = data["nodes"]
            adj_raw = data["adjacency"]
        else:
            # Assume old format (just adjacency)
            coords = {}
            adj_raw = data

        # Normalize adjacency - ensure all values are floats
        normalized = {}
        for u, neighbors in adj_raw.items():
            normalized[u] = {}
            for v, val in neighbors.items():
                if isinstance(val, (int, float)):
                    normalized[u][v] = float(val)
                elif isinstance(val, dict):
                    normalized[u][v] = float(val.get("cost", 1.0))
        
        return cls(normalized, coords)

    def neighbors(self, node: str) -> Dict[str, float]:
        """
        Get all neighbors and edge costs for a given node.
        
        Args:
            node: Node identifier
            
        Returns:
            Dictionary of neighbors with costs
        """
        return self.adj.get(node, {})

    def nodes(self) -> List[str]:
        """
        Get all node identifiers.
        
        Returns:
            List of node names
        """
        return list(self.adj.keys())

    def get_turn_action(self, prev: str, curr: str, next_node: str) -> str:
        """
        Calculate turn direction based on coordinates of 3 consecutive nodes.
        Uses cross product to determine left/right/straight turn.
        
        Args:
            prev: Previous node
            curr: Current node
            next_node: Next node in path
            
        Returns:
            "left", "right", or "straight"
        """
        if not self.node_coords:
            return "straight"
            
        if prev not in self.node_coords or curr not in self.node_coords or next_node not in self.node_coords:
            return "straight"

        p1 = self.node_coords[prev]
        p2 = self.node_coords[curr]
        p3 = self.node_coords[next_node]

        # Vector 1: prev -> curr
        v1x = p2[0] - p1[0]
        v1y = p2[1] - p1[1]

        # Vector 2: curr -> next
        v2x = p3[0] - p2[0]
        v2y = p3[1] - p2[1]

        # Cross product (2D) = v1x*v2y - v1y*v2x
        # Positive = Left turn, Negative = Right turn, Near-zero = Straight
        cross_product = v1x * v2y - v1y * v2x
        
        threshold = 0.1  # Tolerance for straight detection
        
        if cross_product > threshold:
            return "left"
        elif cross_product < -threshold:
            return "right"
        else:
            return "straight"

    def get_distance_between(self, node1: str, node2: str) -> Optional[float]:
        """
        Get Euclidean distance between two nodes (if coordinates available).
        
        Args:
            node1: First node
            node2: Second node
            
        Returns:
            Distance or None if coordinates unavailable
        """
        if not self.node_coords or node1 not in self.node_coords or node2 not in self.node_coords:
            return None
        
        p1 = self.node_coords[node1]
        p2 = self.node_coords[node2]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        return math.sqrt(dx * dx + dy * dy)

    def get_heading_to_node(self, from_node: str, to_node: str) -> Optional[float]:
        """
        Get compass heading (in degrees) from one node to another.
        0° = North (positive Y), 90° = East (positive X)
        
        Args:
            from_node: Starting node
            to_node: Target node
            
        Returns:
            Heading in degrees (0-360) or None if coordinates unavailable
        """
        if not self.node_coords or from_node not in self.node_coords or to_node not in self.node_coords:
            return None
        
        p1 = self.node_coords[from_node]
        p2 = self.node_coords[to_node]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        # Calculate angle in radians, then convert to degrees
        angle_rad = math.atan2(dx, dy)
        angle_deg = math.degrees(angle_rad)
        
        # Normalize to 0-360
        if angle_deg < 0:
            angle_deg += 360
        
        return angle_deg
