# ArUco Marker Detection for PiDrone

This package provides ArUco marker detection and pose estimation for the PiDrone, converted from C++ to Python. It can work with compressed image topics from the Raspberry Pi camera.

## Features

- Detect individual ArUco markers
- Estimate marker poses
- Work with marker maps for localization
- Generate marker maps
- Support for compressed image topics

## Dependencies

- ROS (tested with Noetic)
- OpenCV (with ArUco module)
- NumPy
- TF2

## Usage

### 1. Launch the ArUco detection system

```bash
roslaunch aruco_pose aruco.launch
```

Parameters:
- `dictionary`: ArUco dictionary ID (default: 2, which is DICT_4X4_250)
- `length`: Marker side length in meters (default: 0.15)
- `image_topic`: Camera image topic (default: /raspicam_node/image/compressed)
- `camera_info_topic`: Camera calibration topic (default: /raspicam_node/camera_info)
- `map_file`: Path to the map file (optional)

### 2. Generate a marker map

```bash
python3 scripts/aruco/src/genmap.py --rows 4 --cols 4 --marker-size 0.15 --marker-spacing 0.05 --output map.txt --image
```

Parameters:
- `--rows`: Number of rows in the grid
- `--cols`: Number of columns in the grid
- `--marker-size`: Marker side length in meters
- `--marker-spacing`: Spacing between markers in meters
- `--first-id`: ID of the first marker (default: 0)
- `--dictionary`: ArUco dictionary ID (default: 2)
- `--output`: Output file path
- `--image`: Generate an image of the map
- `--image-path`: Path to save the image (default: map.png)

### 3. Print the markers

Print the generated map image (map.png) and mount it on a flat surface. Make sure to print it at the correct scale so that the markers have the specified size.

## Topics

### ArUco Detector Node

#### Subscribed Topics
- `image_topic` (sensor_msgs/CompressedImage or sensor_msgs/Image): Camera image
- `camera_info` (sensor_msgs/CameraInfo): Camera calibration

#### Published Topics
- `aruco_detect/markers` (aruco_pose/MarkerArray): Detected markers
- `aruco_detect/debug` (sensor_msgs/Image): Debug image with detected markers
- `aruco_detect/visualization` (visualization_msgs/MarkerArray): Visualization markers for RViz

### ArUco Map Node

#### Subscribed Topics
- `image_raw` (sensor_msgs/CompressedImage or sensor_msgs/Image): Camera image
- `camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `markers` (aruco_pose/MarkerArray): Detected markers

#### Published Topics
- `aruco_map/pose` (geometry_msgs/PoseWithCovarianceStamped): Estimated camera pose relative to the map
- `aruco_map/debug` (sensor_msgs/Image): Debug image with detected markers and map
- `aruco_map/visualization` (visualization_msgs/MarkerArray): Visualization markers for RViz

## TF Frames

- `aruco_XX`: Individual marker frames, where XX is the marker ID
- `aruco_map`: Map frame (if using a map)

## Example

1. Generate a 2x2 marker map:
   ```bash
   python3 scripts/aruco/src/genmap.py --rows 2 --cols 2 --marker-size 0.15 --marker-spacing 0.05 --output map.txt --image
   ```

2. Print the generated map.png file.

3. Launch the ArUco detection with the map:
   ```bash
   roslaunch aruco_pose aruco.launch map_file:=map.txt
   ```

4. Visualize in RViz:
   - Add TF display to see the marker frames
   - Add PoseWithCovariance display for the map pose
   - Add Image display for the debug image
   - Add MarkerArray display for the visualization markers
