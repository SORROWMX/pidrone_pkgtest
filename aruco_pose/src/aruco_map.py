#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Detecting and pose estimation of ArUco markers maps
Based on the C++ implementation from aruco_map.cpp
"""

import os
import math
import rospy
import tf2_ros
import numpy as np
import cv2
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from aruco_pose.msg import Marker as ArucoMarker
from aruco_pose.msg import MarkerArray as ArucoMarkerArray
from dynamic_reconfigure.server import Server
from aruco_pose.cfg import MapConfig

from aruco_utils import fill_pose, fill_transform, apply_vertical, transform_to_pose
from aruco_draw import draw_planar_board, draw_axis

class ArucoMap:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_map', anonymous=True)
        
        # Get parameters
        self.enabled = True
        self.dictionary_id = rospy.get_param('~dictionary', 2)
        self.transform_timeout = rospy.Duration(rospy.get_param('~transform_timeout', 0.02))
        
        self.type = rospy.get_param('~type', 'map')
        self.frame_id = rospy.get_param('~frame_id', 'aruco_map')
        self.known_vertical = rospy.get_param('~known_vertical', rospy.get_param('~known_tilt', ''))
        self.flip_vertical = rospy.get_param('~flip_vertical', False)
        self.auto_flip = rospy.get_param('~auto_flip', False)
        self.image_width = rospy.get_param('~image_width', 2000)
        self.image_height = rospy.get_param('~image_height', 2000)
        self.image_margin = rospy.get_param('~image_margin', 200)
        self.image_axis = rospy.get_param('~image_axis', True)
        self.put_markers_count_to_covariance = rospy.get_param('~put_markers_count_to_covariance', False)
        self.markers_parent_frame = rospy.get_param('~markers/frame_id', self.frame_id)
        self.markers_frame = rospy.get_param('~markers/child_frame_id_prefix', '')
        
        # Initialize variables
        self.camera_matrix = np.zeros((3, 3), dtype=np.float64)
        self.dist_coeffs = None
        self.transform = TransformStamped()
        self.transform.child_frame_id = self.frame_id
        self.pose = PoseWithCovarianceStamped()
        self.markers_transforms = []
        self.markers = ArucoMarkerArray()
        self.vis_array = MarkerArray()
        
        # Initialize ArUco board
        self.board = cv2.aruco.Board_create([], cv2.aruco.getPredefinedDictionary(self.dictionary_id))
        
        # Load map
        if self.type == 'map':
            self.map_path = rospy.get_param('~map', '')
            self.load_map(self.map_path)
        elif self.type == 'gridboard':
            self.create_grid_board()
        else:
            rospy.logerr(f"Unknown type: {self.type}")
            rospy.signal_shutdown("Invalid type parameter")
            return
        
        # Set up ROS publishers/subscribers
        self.bridge = CvBridge()
        
        # Publishers
        self.img_pub = rospy.Publisher('~image', Image, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher('~pose', PoseWithCovarianceStamped, queue_size=1)
        self.markers_pub = rospy.Publisher('~map', ArucoMarkerArray, queue_size=1, latch=True)
        self.vis_markers_pub = rospy.Publisher('~visualization', MarkerArray, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = message_filters.Subscriber('image_raw', Image)
        self.info_sub = message_filters.Subscriber('camera_info', CameraInfo)
        self.markers_sub = message_filters.Subscriber('markers', ArucoMarkerArray)
        
        # Synchronize subscribers
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub, self.markers_sub], 10, 0.1)
        self.sync.registerCallback(self.callback)
        
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Dynamic reconfigure
        self.dyn_srv = Server(MapConfig, self.param_callback)
        
        # Publish initial map data
        self.publish_map()
        
        rospy.loginfo("ArUco map ready")
    
    def load_map(self, filename):
        """Load markers map from file"""
        self.clear_markers()
        
        if not filename:
            rospy.loginfo("No map loaded")
            return
        
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    
                    # Skip empty lines
                    if not line:
                        continue
                    
                    # Skip comments
                    if line[0] == '#':
                        rospy.logdebug(f"Skipping line as a comment: {line}")
                        continue
                    
                    # Parse line
                    parts = line.split()
                    
                    # Check if we have enough data
                    if len(parts) < 4:
                        rospy.logerr(f"Not enough data in line: {line}; "
                                    f"Each marker must have at least id, length, x, y fields")
                        continue
                    
                    # Parse marker data
                    marker_id = int(parts[0])
                    length = float(parts[1])
                    x = float(parts[2])
                    y = float(parts[3])
                    
                    # Optional parameters
                    z = float(parts[4]) if len(parts) > 4 else 0.0
                    yaw = float(parts[5]) if len(parts) > 5 else 0.0
                    pitch = float(parts[6]) if len(parts) > 6 else 0.0
                    roll = float(parts[7]) if len(parts) > 7 else 0.0
                    
                    # Add marker to the map
                    self.add_marker(marker_id, length, x, y, z, yaw, pitch, roll)
            
            rospy.loginfo(f"Loading {filename} complete ({len(self.board.ids)} markers)")
        except Exception as e:
            rospy.logerr(f"Error loading map: {e}")
            self.clear_markers()
    
    def create_grid_board(self):
        """Create a grid board of markers"""
        rospy.loginfo("Generate gridboard")
        rospy.logwarn("Gridboard maps are deprecated")
        
        markers_x = rospy.get_param('~markers_x', 10)
        markers_y = rospy.get_param('~markers_y', 10)
        first_marker = rospy.get_param('~first_marker', 0)
        
        if not rospy.has_param('~markers_side'):
            rospy.logerr("Required param ~markers_side is not defined")
            rospy.signal_shutdown("Missing required parameter")
            return
        markers_side = rospy.get_param('~markers_side')
        
        if not rospy.has_param('~markers_sep_x'):
            rospy.logerr("Required param ~markers_sep_x is not defined")
            rospy.signal_shutdown("Missing required parameter")
            return
        markers_sep_x = rospy.get_param('~markers_sep_x')
        
        if not rospy.has_param('~markers_sep_y'):
            rospy.logerr("Required param ~markers_sep_y is not defined")
            rospy.signal_shutdown("Missing required parameter")
            return
        markers_sep_y = rospy.get_param('~markers_sep_y')
        
        # Get marker IDs
        marker_ids = rospy.get_param('~marker_ids', [])
        if marker_ids:
            if len(marker_ids) != markers_x * markers_y:
                rospy.logerr("~marker_ids length should be equal to ~markers_x * ~markers_y")
                rospy.signal_shutdown("Invalid marker_ids parameter")
                return
        else:
            # Fill marker_ids automatically
            marker_ids = list(range(first_marker, first_marker + markers_x * markers_y))
        
        # Create markers
        max_y = markers_y * markers_side + (markers_y - 1) * markers_sep_y
        for y in range(markers_y):
            for x in range(markers_x):
                x_pos = x * (markers_side + markers_sep_x)
                y_pos = max_y - y * (markers_side + markers_sep_y) - markers_side
                marker_id = marker_ids[y * markers_y + x]
                rospy.loginfo(f"Add marker {marker_id} {x_pos} {y_pos}")
                self.add_marker(marker_id, markers_side, x_pos, y_pos, 0, 0, 0, 0)
    
    def clear_markers(self):
        """Clear all markers"""
        self.board.ids = []
        self.board.objPoints = []
        self.markers.markers = []
        self.vis_array.markers = []
        self.markers_transforms = []
    
    def add_marker(self, marker_id, length, x, y, z, yaw, pitch, roll):
        """Add a marker to the map"""
        # Check if marker ID is in range for current dictionary
        num_markers = self.board.dictionary.bytesList.shape[0]
        if num_markers <= marker_id:
            rospy.logerr(f"Marker id {marker_id} is not in dictionary; current dictionary contains {num_markers} markers.")
            return
        
        # Check if marker is already in the board
        if marker_id in self.board.ids:
            rospy.logerr(f"Marker id {marker_id} is already in the map")
            return
        
        # Create transform
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        transform = tf.transformations.compose_matrix(
            angles=tf.transformations.euler_from_quaternion(q),
            translate=[x, y, z]
        )
        
        # Calculate marker corners
        half_len = length / 2
        corners_3d = np.array([
            [-half_len, half_len, 0],
            [half_len, half_len, 0],
            [half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ])
        
        # Transform corners
        corners_transformed = []
        for corner in corners_3d:
            # Add homogeneous coordinate
            corner_h = np.append(corner, 1)
            # Apply transform
            corner_transformed = np.dot(transform, corner_h)
            # Convert back to 3D point
            corners_transformed.append(corner_transformed[:3])
        
        # Add marker to board
        self.board.ids.append(marker_id)
        self.board.objPoints.append(np.array(corners_transformed, dtype=np.float32))
        
        # Add marker's static transform
        if self.markers_frame:
            marker_transform = TransformStamped()
            marker_transform.header.frame_id = self.markers_parent_frame
            marker_transform.child_frame_id = f"{self.markers_frame}{marker_id}"
            marker_transform.transform.translation.x = x
            marker_transform.transform.translation.y = y
            marker_transform.transform.translation.z = z
            marker_transform.transform.rotation.x = q[0]
            marker_transform.transform.rotation.y = q[1]
            marker_transform.transform.rotation.z = q[2]
            marker_transform.transform.rotation.w = q[3]
            self.markers_transforms.append(marker_transform)
        
        # Add marker to array
        marker = ArucoMarker()
        marker.id = marker_id
        marker.length = length
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        self.markers.markers.append(marker)
        
        # Add visualization marker
        vis_marker = Marker()
        vis_marker.header.frame_id = self.frame_id
        vis_marker.action = Marker.ADD
        vis_marker.id = len(self.vis_array.markers)
        vis_marker.ns = "aruco_map_marker"
        vis_marker.type = Marker.CUBE
        vis_marker.scale.x = length
        vis_marker.scale.y = length
        vis_marker.scale.z = 0.001
        vis_marker.color.r = 1.0
        vis_marker.color.g = 0.5
        vis_marker.color.b = 0.5
        vis_marker.color.a = 0.8
        vis_marker.pose.position.x = x
        vis_marker.pose.position.y = y
        vis_marker.pose.position.z = z
        vis_marker.pose.orientation.x = q[0]
        vis_marker.pose.orientation.y = q[1]
        vis_marker.pose.orientation.z = q[2]
        vis_marker.pose.orientation.w = q[3]
        vis_marker.frame_locked = True
        self.vis_array.markers.append(vis_marker)
    
    def publish_map(self):
        """Publish map data"""
        self.publish_markers_frames()
        self.publish_markers()
        self.publish_map_image()
        self.vis_markers_pub.publish(self.vis_array)
    
    def publish_markers_frames(self):
        """Publish marker transforms"""
        if self.markers_transforms:
            self.static_br.sendTransform(self.markers_transforms)
    
    def publish_markers(self):
        """Publish markers array"""
        self.markers_pub.publish(self.markers)
    
    def publish_map_image(self):
        """Create and publish map image"""
        size = (self.image_width, self.image_height)
        
        if self.board.ids:
            # Draw board
            image = draw_planar_board(self.board, size, self.image_margin, 1, self.image_axis)
            encoding = 'rgb8' if self.image_axis else 'mono8'
        else:
            # Empty map
            image = np.ones((size[1], size[0]), dtype=np.uint8) * 255
            encoding = 'mono8'
        
        # Publish image
        msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        self.img_pub.publish(msg)
    
    def align_obj_points_to_center(self, obj_points):
        """Align object points to the center of mass"""
        # Calculate center of mass
        center_x = np.mean(obj_points[:, 0])
        center_y = np.mean(obj_points[:, 1])
        center_z = np.mean(obj_points[:, 2])
        
        # Align points to center
        obj_points[:, 0] -= center_x
        obj_points[:, 1] -= center_y
        obj_points[:, 2] -= center_z
        
        return center_x, center_y, center_z
    
    def callback(self, image_msg, cinfo_msg, markers_msg):
        """Process incoming data"""
        if not self.enabled:
            return
        
        if not markers_msg.markers:
            return  # No markers detected
        
        # Parse camera info
        self.camera_matrix = np.reshape(cinfo_msg.K, (3, 3))
        self.dist_coeffs = np.array(cinfo_msg.D)
        
        # Extract marker data
        ids = []
        corners = []
        
        for marker in markers_msg.markers:
            ids.append(marker.id)
            marker_corners = np.array([
                [marker.c1.x, marker.c1.y],
                [marker.c2.x, marker.c2.y],
                [marker.c3.x, marker.c3.y],
                [marker.c4.x, marker.c4.y]
            ], dtype=np.float32)
            corners.append(marker_corners)
        
        # Set covariance if needed
        if self.put_markers_count_to_covariance:
            # Count valid markers (markers that are in the map)
            valid_markers = sum(1 for marker in markers_msg.markers if marker.id in self.board.ids)
            self.pose.pose.covariance[0] = valid_markers
        
        # Estimate board pose
        if self.known_vertical.empty():
            # Simple estimation
            retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners, ids, self.board, self.camera_matrix, self.dist_coeffs, None, None, False)
            
            if not retval:
                # Failed to estimate pose
                self.publish_debug(image_msg, corners, ids)
                return
            
            # Fill transform and pose
            self.transform.header.stamp = markers_msg.header.stamp
            self.transform.header.frame_id = markers_msg.header.frame_id
            self.pose.header = self.transform.header
            
            fill_pose(self.pose.pose.pose, rvec, tvec)
            fill_transform(self.transform.transform, rvec, tvec)
        else:
            # Estimation with known vertical
            obj_points, img_points = cv2.aruco.getBoardObjectAndImagePoints(
                self.board, corners, ids)
            
            if obj_points.size == 0:
                # No points to estimate pose
                self.publish_debug(image_msg, corners, ids)
                return
            
            # Align object points to center
            center_x, center_y, center_z = self.align_obj_points_to_center(obj_points)
            
            # Estimate pose
            retval, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, self.camera_matrix, self.dist_coeffs)
            
            if not retval:
                # Failed to estimate pose
                self.publish_debug(image_msg, corners, ids)
                return
            
            # Fill transform
            fill_transform(self.transform.transform, rvec, tvec)
            
            # Apply vertical orientation if available
            try:
                vertical = self.tf_buffer.lookup_transform(
                    markers_msg.header.frame_id, self.known_vertical,
                    markers_msg.header.stamp, self.transform_timeout)
                
                apply_vertical(
                    self.transform.transform.rotation, vertical.transform.rotation,
                    self.flip_vertical, self.auto_flip)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(1, f"Can't retrieve known vertical: {e}")
            
            # Apply shift to center
            shift = TransformStamped()
            shift.transform.translation.x = -center_x
            shift.transform.translation.y = -center_y
            shift.transform.translation.z = -center_z
            shift.transform.rotation.w = 1.0
            
            # Apply shift transform
            self.transform = tf2_ros.do_transform_transform(shift, self.transform)
            
            # Update tvec for debug visualization
            tvec[0] = self.transform.transform.translation.x
            tvec[1] = self.transform.transform.translation.y
            tvec[2] = self.transform.transform.translation.z
            
            # Set header
            self.transform.header.stamp = markers_msg.header.stamp
            self.transform.header.frame_id = markers_msg.header.frame_id
            self.pose.header = self.transform.header
            
            # Convert transform to pose
            transform_to_pose(self.transform.transform, self.pose.pose.pose)
        
        # Broadcast transform
        if self.transform.child_frame_id:
            self.br.sendTransform(self.transform)
        
        # Publish pose
        self.pose_pub.publish(self.pose)
        
        # Publish debug image
        self.publish_debug(image_msg, corners, ids, rvec, tvec)
    
    def publish_debug(self, image_msg, corners, ids, rvec=None, tvec=None):
        """Publish debug image with detected markers"""
        if self.debug_pub.get_num_connections() == 0:
            return
        
        # Convert image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        # Draw board axis if pose was estimated
        if rvec is not None and tvec is not None:
            draw_axis(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 1.0)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        debug_msg.header.frame_id = image_msg.header.frame_id
        debug_msg.header.stamp = image_msg.header.stamp
        self.debug_pub.publish(debug_msg)
    
    def param_callback(self, config, level):
        """Dynamic reconfigure callback"""
        self.enabled = config.enabled
        
        if self.type == 'map' and config.map != self.map_path:
            self.map_path = config.map
            self.load_map(self.map_path)
            self.publish_map()
        
        if config.image_axis != self.image_axis:
            self.image_axis = config.image_axis
            self.publish_map_image()
        
        return config


if __name__ == '__main__':
    try:
        aruco_map = ArucoMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 