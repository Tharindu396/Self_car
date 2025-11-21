# RPI Car Navigation System - Usage Guide

This navigation system provides real-time position tracking, Uber-style path visualization, and junction decision making for your RPI car using Dijkstra's algorithm.

## Features

✅ **Real-time Position Tracking** - Track your car's position as it moves along the route  
✅ **Uber-style Visualization** - Visual path display showing car, rider, and route  
✅ **Junction Decision Making** - Automatic detection and instructions at intersections  
✅ **Dijkstra Pathfinding** - Optimal route calculation using graph-based navigation  
✅ **Integration Ready** - Works with your existing lane change planner  

## Quick Start

### 1. Basic Navigation

```python
from navigation_system import NavigationSystem

# Create navigation system
nav = NavigationSystem()

# Set route from start to destination
nav.set_route("A", "D")

# Update position as car moves (speed in units/second)
nav.update_position(speed=0.5, dt=0.1)

# Get current position
print(nav.car_position)

# Get remaining distance
print(f"Remaining: {nav.get_remaining_distance():.2f} units")
```

### 2. With Visualization (Uber-style)

```python
from navigation_system import NavigationSystem, NavigationVisualizer

nav = NavigationSystem()
nav.set_route("A", "D")

# Create visualizer
viz = NavigationVisualizer(nav)

# Simulate movement and visualize
speed = 0.5
while nav.get_remaining_distance() > 0.1:
    nav.update_position(speed, dt=0.1)
    viz.update_and_draw()  # Updates the visualization
    time.sleep(0.1)
```

### 3. Junction Decision Making

```python
# Check for junction decisions
junction = nav.get_junction_decision()

if junction:
    print(f"Instruction: {junction.instruction}")
    print(f"Direction: {junction.direction}")  # left, right, straight, back
    print(f"Distance: {junction.distance_to_junction:.2f} units")
```

### 4. Integration with DonkeyCar

Use `integrated_navigation.py` to combine with your lane planner:

```python
from integrated_navigation import IntegratedNavigationController

# In your manage.py or vehicle setup:
nav_controller = IntegratedNavigationController()

# Start navigation
nav_controller.start_navigation("A", "D")

# In your vehicle loop, it combines navigation + lane planning:
angle, throttle, nav_info = nav_controller.update(
    lane_offset, lane_heading, detections, dt=0.1
)

# Get junction instructions
instruction = nav_controller.get_junction_instruction()
```

## Visualization Features

The visualization shows:

- **Blue Line**: Planned route (like Uber's route line)
- **Red Triangle**: Your car's current position (points in direction of travel)
- **Green Circle**: Rider/destination location
- **Orange Circles**: Junction nodes
- **Yellow Boxes**: Junction instructions (appears when approaching)
- **Gray Lines**: All available roads in the graph
- **Info Panel**: Current position, remaining distance, speed, etc.

## Graph Structure

Your `graph.json` defines the road network:

- **Nodes**: Road segments (A, B, C, D, E, F, G) and Junctions (J1, J2, J3, J4)
- **Edges**: Connections between nodes with weights (distances)

Example:
```json
{
    "A": {"E": 4, "J1": 1},
    "J1": {"A": 1, "J3": 5, "J2": 3},
    ...
}
```

## Examples

Run the example scripts:

```bash
# Basic navigation demo
python navigation_example.py basic

# With visualization
python navigation_example.py viz

# Multiple routes
python navigation_example.py routes

# Junction decisions
python navigation_example.py junctions
```

Or run the full demo:
```bash
python navigation_system.py
```

## Position Tracking

The system tracks:
- **Current Node**: Which node/road segment the car is on
- **Progress**: How far along the current edge (0.0 to 1.0)
- **Heading**: Direction of travel in degrees
- **Speed**: Current speed in units/second

## Junction Handling

When approaching a junction, the system:
1. Detects proximity to junction
2. Determines required direction (left/right/straight/back)
3. Provides distance to junction
4. Generates human-readable instructions

Example output:
```
🚦 In 0.3 units: Turn right
   Distance: 0.30 units
   Direction: right
```

## Customization

### Adjust Speed
```python
nav.update_position(speed=1.0, dt=0.1)  # Faster
nav.update_position(speed=0.2, dt=0.1)  # Slower
```

### Custom Coordinates
Modify `_get_node_coordinates()` in `NavigationVisualizer` to use actual GPS coordinates or map positions.

### Enhanced Heading Calculation
Update `_calculate_heading()` in `NavigationSystem` to use actual road geometry or sensor data.

## Integration Tips

1. **Position Updates**: Call `update_position()` regularly (every 100ms recommended)
2. **Speed Mapping**: Map your throttle/speed sensor to units that match your graph weights
3. **Junction Detection**: Use `get_junction_decision()` to show turn indicators
4. **Visualization**: Run visualization on a separate thread if needed for real-time updates

## Troubleshooting

**No path found?**
- Check that start and destination nodes exist in `graph.json`
- Verify the graph is connected (all nodes reachable)

**Position not updating?**
- Ensure `update_position()` is called regularly
- Check that speed > 0
- Verify route is set with `set_route()`

**Visualization not showing?**
- Install matplotlib: `pip install matplotlib`
- Check that nodes have valid coordinates

## Next Steps

- Add GPS integration for real-world coordinates
- Enhance junction detection with camera/vision
- Add turn-by-turn voice instructions
- Integrate with mapping services (OpenStreetMap, etc.)


