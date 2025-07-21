# ArUco Marker Detection for ROS

This package provides Python implementation for ArUco marker detection and pose estimation in ROS.

## Features

- Detection of ArUco markers in images
- Pose estimation of detected markers
- Support for marker length override (different sizes for different markers)
- Map loading from files
- Dynamic reconfiguration support
- ROS service for map reloading

## Usage

### Basic Usage

Launch the node:

```bash
rosrun <your_package> recognition_of_aruco_marker_node.py
```

### Parameters

The node accepts the following parameters:

- `~map_path`: Path to the map file (optional)
- `~frame_id_prefix`: Prefix for the TF frames of detected markers (default: "aruco_")
- `~send_tf`: Whether to publish TF transforms for detected markers (default: true)

### Map File Format

The map file should contain one marker per line with the following format:

```
marker_id marker_length [x y z yaw pitch roll]
```

Where:
- `marker_id`: Integer ID of the marker
- `marker_length`: Size of the marker in meters
- `x y z yaw pitch roll`: Optional position and orientation (not used for detection, but can be used for visualization)

Lines starting with `#` are treated as comments.

Example:
```
# This is a comment
0 0.05 0.0 0.0 0.0 0.0 0.0 0.0
1 0.1 0.5 0.0 0.0 0.0 0.0 0.0
```

### Topics

The node subscribes to:
- `/drone_vision/image`: Input image from camera

The node publishes to:
- `/drone_vision/image_with_marks`: Image with detected markers
- `/drone_vision/markers_tf`: Transforms of detected markers

### Services

- `~reload_map`: Reload the map file (no parameters)

### Dynamic Reconfiguration

The node supports dynamic reconfiguration of the following parameters:
- `enabled`: Enable/disable marker detection
- `length`: Default marker size in meters
- Various detector parameters (adaptiveThreshConstant, adaptiveThreshWinSizeMin, etc.)

## Comparison with C++ Implementation

This Python implementation provides similar functionality to the C++ implementation (`aruco_detect.cpp` and `aruco_map.cpp`) with some differences:

1. **Language**: Python vs C++
2. **Performance**: The C++ implementation may be faster for real-time applications
3. **Features**: The Python implementation includes the core functionality but lacks some advanced features like visualization markers for RViz

## Dependencies

- ROS
- OpenCV with ArUco module
- Python 3
- cv_bridge
- dynamic_reconfigure (optional)

## License

This software is released under the MIT license. 