# mocap_px4_bridge

A ROS 2 bridge that connects motion capture (mocap) systems to PX4 autopilot via uXRCE-DDS. This package enables real-time vehicle positioning and odometry updates from mocap systems (e.g., Qualisys) to PX4-based UAVs.

## Overview

The `mocap_px4_bridge` package provides a middleware layer that:
- Subscribes to mocap pose data (typically from Qualisys or similar systems)
- Performs coordinate frame transformations (ENU to NED)
- Converts pose information to PX4's `VehicleOdometry` message format
- Publishes the transformed data to PX4 via the uXRCE-DDS bridge

This enables PX4's Extended Kalman Filter (EKF) to use motion capture data for improved localization and state estimation.

## Dependencies

### ROS 2 Dependencies
- `rclpy` - ROS 2 Python client library
- `px4_msgs` - PX4 message definitions
- `geometry_msgs` - ROS geometry message definitions

### System Requirements
- ROS 2 (tested on Humble/Iron)
- PX4 firmware with uXRCE-DDS bridge enabled
- Motion capture system publishing PoseStamped messages

## Package Contents

```
mocap_px4_bridge/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package manifest
├── scripts/
│   └── mocap_bridge_node.py # Main node implementation
└── include/
    └── mocap_px4_bridge/    # Header files (Python package)
```

## Usage

### 1. Installation

Build the package using colcon:

```bash
cd ~/px4_ws
colcon build --packages-select mocap_px4_bridge
source install/setup.bash
```

### 2. Running the Node

Launch the bridge node:

```bash
ros2 run mocap_px4_bridge mocap_bridge_node.py
```

Or use a launch file:

```bash
ros2 launch mocap_px4_bridge mocap_bridge.launch.py
```

### 3. Configuration

The node supports the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_id` | string | `/pv162` | Vehicle identifier (mocap system name) |

Pass parameters at launch:

```bash
ros2 run mocap_px4_bridge mocap_bridge_node.py --ros-args -p vehicle_id:=/my_drone
```

## Coordinate System Transformation

The bridge performs critical coordinate system conversions:

### Input: ENU Frame (Eastern-Northern-Up)
- From mocap system (e.g., Qualisys)
- X: East, Y: North, Z: Up

### Output: NED Frame (North-East-Down)
- Required by PX4
- X: North, Y: East, Z: Down

### Transformation Details

**Position Conversion:**
- North (NED X) ← East (ENU Y)
- East (NED Y) ← North (ENU X)
- Down (NED Z) ← -Up (ENU Z)

**Orientation Conversion:**
Quaternion components are reordered and negated as needed to maintain proper orientation in NED frame.

## Topics

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{vehicle_id}/pose` | `geometry_msgs/PoseStamped` | Mocap pose data in ENU frame |

Example: `/pv162/pose`

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | Odometry for PX4 EKF (NED frame) |

## PX4 Configuration

Ensure PX4 is configured to accept visual odometry:

### Required Parameters
- `EKF2_AID_MASK`: Set bit for external vision (typically: 32)
- `EKF2_EV_POS_X/Y/Z`: Position of vision system sensor relative to vehicle center
- `MAV_0_CONFIG`: Serial/network configuration for uXRCE-DDS

### Example PX4 Setup (QGroundControl)
1. Enable uXRCE-DDS: `UXRCE_DDS_CFG` = 1 (TELEM2)
2. Enable visual odometry in EKF2
3. Set appropriate variance parameters

## Key Implementation Details

### Zero-Timestamp Trick
The node sets `timestamp` and `timestamp_sample` to 0, which triggers special handling in PX4's visual odometry processing. This is crucial for Mavlink Inspector visibility and proper data handling.

### Variance Configuration
Variance values are set to prevent the EKF from rejecting data:
- Position variance: 0.01 m²
- Orientation variance: 0.02 rad²

These values should be tuned based on your mocap system's accuracy.

### QoS Settings
The node uses a `BEST_EFFORT` QoS profile with:
- Reliability: Best Effort
- Durability: Volatile
- History: Keep Last (depth: 1)

This ensures low-latency communication suitable for real-time control.

## Troubleshooting

### Node Not Receiving Pose Data
- Verify mocap system is publishing to the correct topic: `ros2 topic list | grep pose`
- Check vehicle_id parameter matches mocap system naming
- Ensure PoseStamped messages are being published: `ros2 topic echo {vehicle_id}/pose`

### PX4 Not Receiving Odometry
- Verify uXRCE-DDS bridge is running: `ros2 node list` should show `/uxrce_dds_client`
- Check PX4 parameters: `EKF2_AID_MASK` should have external vision bit enabled
- Monitor published messages: `ros2 topic echo /fmu/in/vehicle_visual_odometry`

### Coordinate Transformation Issues
- Verify mocap frame is ENU (check system documentation)
- PX4 requires NED frame (conversion is handled automatically)
- If vehicle orientation seems incorrect, check quaternion transformation

## Related Packages

- **mocap_gps_offboard**: Alternative bridge providing GPS-like positioning
- **qualisys_mocap**: Qualisys motion capture integration
- **px4_msgs**: PX4 message definitions

## References

- [PX4 Visual Odometry Documentation](https://docs.px4.io/main/en/sensor/visual_odometry.html)
- [ROS 2 Documentation](https://docs.ros.org/)
- [uXRCE-DDS Documentation](https://micro-xrce-dds.docs.eprosima.com/)

## Author

Worawis Sribunma (worawissribunma@gmail.com)
