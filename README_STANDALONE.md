# Standalone Navigation System

This navigation system works **independently** without requiring `planner_part.py`, `lane_det.py`, or `manage.py`.

## Main Components

1. **`dijikstra_algo.py`** - Main Dijkstra pathfinding algorithm with OCR
2. **`standalone_navigation.py`** - Navigation system with position tracking
3. **`main_navigation.py`** - Main entry point that integrates everything

## Quick Start

### Option 1: Full Navigation System

```bash
python main_navigation.py
```

This will:
1. Use OCR to detect your start position from camera
2. Ask for destination
3. Calculate route using Dijkstra
4. Show real-time navigation with visualization

### Option 2: Quick Pathfinding Only

```bash
python main_navigation.py quick
```

Or use directly:

```bash
python dijikstra_algo.py
```

## How It Works

### 1. Dijkstra Algorithm (`dijikstra_algo.py`)

- Loads graph from `graph.json`
- Uses OCR to detect start node from camera
- Calculates shortest path using Dijkstra's algorithm
- No dependencies on other modules

### 2. Navigation System (`standalone_navigation.py`)

- Tracks car position in real-time
- Updates position based on speed
- Provides junction decisions
- Shows Uber-style visualization
- **No dependencies** on planner_part, lane_det, or manage

### 3. Main Integration (`main_navigation.py`)

- Combines Dijkstra pathfinding with navigation
- Handles OCR detection
- Provides visualization or text-only modes

## Usage Examples

### Basic Pathfinding

```python
from dijikstra_algo import dijkstra, graph

path, distance = dijkstra(graph, "A", "D")
print(f"Path: {' -> '.join(path)}")
print(f"Distance: {distance}")
```

### Full Navigation

```python
from standalone_navigation import StandaloneNavigation

nav = StandaloneNavigation()
nav.set_route("A", "D")

# Update position as car moves
nav.update_position(speed=0.5, dt=0.1)

# Get junction decisions
junction = nav.get_junction_decision()
if junction:
    print(junction.instruction)
```

### With Visualization

```python
from standalone_navigation import StandaloneNavigation, NavigationVisualizer

nav = StandaloneNavigation()
nav.set_route("A", "D")

viz = NavigationVisualizer(nav)

# Simulate movement
while nav.get_remaining_distance() > 0.1:
    nav.update_position(0.5, dt=0.1)
    viz.update_and_draw()
    time.sleep(0.1)
```

## Key Features

✅ **No Dependencies** - Works without planner_part, lane_det, or manage.py  
✅ **OCR Integration** - Detects start position from camera  
✅ **Dijkstra Pathfinding** - Optimal route calculation  
✅ **Real-time Tracking** - Position updates as car moves  
✅ **Junction Decisions** - Automatic turn instructions  
✅ **Visualization** - Uber-style path display  

## File Structure

```
dijikstra_algo.py          # Main Dijkstra algorithm + OCR
standalone_navigation.py   # Navigation system (no dependencies)
main_navigation.py          # Main entry point
graph.json                  # Road network graph
```

## Differences from Old System

| Old System | New System |
|------------|------------|
| Required planner_part.py | Standalone, no dependencies |
| Required lane_det.py | Not needed |
| Required manage.py | Not needed |
| Complex integration | Simple, direct usage |

## Integration with Your Car

To use with your RPI car:

```python
from standalone_navigation import StandaloneNavigation
from dijikstra_algo import detect_start_node_from_camera

# Initialize
nav = StandaloneNavigation()

# Detect start position
start = detect_start_node_from_camera()
goal = "D"  # Your destination

# Set route
nav.set_route(start, goal)

# In your car control loop:
while True:
    # Get your car's speed (from sensors or throttle)
    speed = get_car_speed()  # Your function
    
    # Update navigation
    nav.update_position(speed, dt=0.1)
    
    # Get junction decision
    junction = nav.get_junction_decision()
    if junction:
        print(f"Turn {junction.direction} at {junction.next_node}")
    
    # Get next waypoint
    next_waypoint = nav.get_next_waypoint()
    
    time.sleep(0.1)
```

## Troubleshooting

**OCR not working?**
- Check camera is connected
- Install pytesseract: `pip install pytesseract`
- Install Tesseract OCR on your system

**No visualization?**
- Install matplotlib: `pip install matplotlib`
- Use text-only mode instead

**Path not found?**
- Check nodes exist in graph.json
- Verify graph is connected (all nodes reachable)

