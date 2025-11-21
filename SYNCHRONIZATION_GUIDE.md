# Car Movement and Navigation Synchronization Guide

## Current Synchronization Flow

### How It Works Now

```
┌─────────────────────────────────────────────────────────────┐
│                    Vehicle Control Loop                      │
│                  (Runs every ~100ms)                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  1. Camera captures image              │
        │     → lane/offset, lane/heading        │
        │     → det/objs (object detections)     │
        └───────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  2. IntegratedNavigationController     │
        │     .update() is called                │
        └───────────────────────────────────────┘
                            │
                            ├──────────────────────────┐
                            ▼                          ▼
        ┌──────────────────────────┐    ┌──────────────────────────┐
        │  Navigation Position     │    │  Lane Planner             │
        │  Update (PROBLEM AREA)   │    │  - Calculates angle      │
        │  - Uses OLD speed        │    │  - Calculates throttle   │
        │  - Updates position      │    │  - Returns controls      │
        └──────────────────────────┘    └──────────────────────────┘
                            │                          │
                            │                          ▼
                            │              ┌──────────────────────────┐
                            │              │  Speed = throttle * 2.0  │
                            │              │  (Too late for nav!)     │
                            │              └──────────────────────────┘
                            │                          │
                            └──────────┬───────────────┘
                                       ▼
                        ┌──────────────────────────┐
                        │  Return:                 │
                        │  - angle (steering)      │
                        │  - throttle              │
                        │  - nav_info              │
                        └──────────────────────────┘
                                       │
                                       ▼
                        ┌──────────────────────────┐
                        │  3. Actuators execute    │
                        │     - Steering motor     │
                        │     - Throttle motor     │
                        └──────────────────────────┘
                                       │
                                       ▼
                        ┌──────────────────────────┐
                        │  4. Car physically moves │
                        │     (Next loop cycle)    │
                        └──────────────────────────┘
```

### The Problem

**Current Issue**: Position is updated using speed calculated from throttle, but throttle is calculated AFTER position update. This creates a one-step delay.

**Timing Problem**:
1. Loop iteration N: Position updated with speed from iteration N-1
2. Throttle calculated for iteration N
3. Speed for iteration N calculated (but used in iteration N+1)

## Improved Synchronization Methods

### Method 1: Sensor-Based Speed (Recommended)

Use actual speed sensors (encoder, IMU, GPS) instead of estimating from throttle:

```python
# With speed sensor
actual_speed = speed_sensor.get_speed()  # m/s or units/s
nav.update_position(actual_speed, dt)
```

### Method 2: Predictive Speed Update

Update position with predicted speed based on current throttle:

```python
# Predict speed from throttle
predicted_speed = throttle_to_speed(throttle)
nav.update_position(predicted_speed, dt)
```

### Method 3: Odometry-Based Position

Use wheel encoders or IMU to directly update position:

```python
# With odometry
distance_traveled = encoder.get_distance()
nav.update_position_from_distance(distance_traveled)
```

### Method 4: GPS-Based Position (Most Accurate)

Use GPS coordinates to directly set position:

```python
# With GPS
gps_coords = gps.get_coordinates()
nav.update_position_from_gps(gps_coords, current_node)
```

## Implementation Details

### Current Code Flow

```python
# In IntegratedNavigationController.update():

# STEP 1: Update navigation (uses OLD speed)
if self.is_navigating:
    base_speed = self.current_speed  # From previous iteration!
    self.nav_system.update_position(base_speed, dt)

# STEP 2: Get lane planner output
angle, throttle = self.lane_planner.run(...)

# STEP 3: Calculate NEW speed (for next iteration)
self.current_speed = throttle * 2.0  # Too late for current update!
```

### Position Update Calculation

```python
# In NavigationSystem.update_position():

distance = speed * dt  # Distance traveled in this time step

# Update progress along current edge
progress += distance / edge_length

# If reached end of edge, move to next node
if progress >= 1.0:
    car_position.node = next_node
    car_position.progress = 0.0
```

## Synchronization Accuracy

### Factors Affecting Sync:

1. **Update Frequency**: How often `update()` is called
   - Recommended: 10-20 Hz (50-100ms intervals)
   - Higher frequency = better accuracy

2. **Speed Measurement**: How speed is determined
   - Throttle-based: ~10-20% error
   - Sensor-based: ~1-5% error
   - GPS-based: ~0.5-2% error

3. **Time Measurement**: Accuracy of `dt`
   - Auto-calculated: Good (uses system time)
   - Manual: Must be accurate

4. **Graph Accuracy**: Road segment lengths
   - Must match real-world distances
   - Calibrate graph weights to actual distances

## Best Practices

1. **Use Real Sensors**: Integrate speed sensors or GPS when possible
2. **Calibrate Speed Mapping**: Test and adjust throttle-to-speed conversion
3. **Regular Updates**: Call update() consistently every loop iteration
4. **Validate Position**: Periodically check position against known landmarks
5. **Handle Errors**: Account for wheel slip, GPS drift, etc.

## Example: Adding Speed Sensor

```python
class SpeedSensor:
    def get_speed(self) -> float:
        # Read from encoder, IMU, or GPS
        return self.current_speed_mps

# In IntegratedNavigationController:
def update(self, ..., speed_sensor=None):
    # Use sensor if available, else estimate
    if speed_sensor:
        actual_speed = speed_sensor.get_speed()
    else:
        actual_speed = self.current_speed
    
    nav.update_position(actual_speed, dt)
```

## Troubleshooting Sync Issues

**Position drifts over time?**
- Check speed calibration
- Verify graph edge lengths match reality
- Add position correction at known nodes

**Position updates too slow/fast?**
- Adjust speed conversion factor
- Check update frequency
- Verify time delta calculation

**Junction decisions too early/late?**
- Adjust distance thresholds
- Calibrate graph distances
- Add safety margins


