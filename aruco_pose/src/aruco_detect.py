#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Detecting and pose estimation of ArUco markers
Based on the C++ implementation from aruco_detect.cpp
"""

import math
import rospy
import tf2_ros
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from aruco_pose.msg import Marker as ArucoMarker
from aruco_pose.msg import MarkerArray as ArucoMarkerArray
from aruco_pose.srv import SetMarkers, SetMarkersResponse
from dynamic_reconfigure.server import Server
from aruco_pose.cfg import DetectorConfig

# Изменяем импорт на относительный путь к модулю в том же пакете
from aruco_pose.src.aruco_utils import fill_pose, fill_transform, apply_vertical, fill_corners, fill_translation


class ArucoDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_detect', anonymous=True)
        
        # Get parameters
        self.enabled = True
        self.dictionary_id = rospy.get_param('~dictionary', 2)
        self.estimate_poses = rospy.get_param('~estimate_poses', True)
        self.send_tf = rospy.get_param('~send_tf', True)
        self.use_map_markers = rospy.get_param('~use_map_markers', False)
        self.waiting_for_map = self.use_map_markers
        
        if self.estimate_poses:
            if not rospy.has_param('~length'):
                rospy.logerr("Can't estimate marker's poses as ~length parameter is not defined")
                rospy.signal_shutdown("Missing required parameter")
                return
            self.length = rospy.get_param('~length')
        
        self.length_override = {}
        self.read_length_override()
        self.transform_timeout = rospy.Duration(rospy.get_param('~transform_timeout', 0.02))
        
        self.known_vertical = rospy.get_param('~known_vertical', rospy.get_param('~known_tilt', ''))
        self.flip_vertical = rospy.get_param('~flip_vertical', False)
        self.auto_flip = rospy.get_param('~auto_flip', False)
        
        self.frame_id_prefix = rospy.get_param('~frame_id_prefix', 'aruco_')
        
        self.camera_matrix = np.zeros((3, 3), dtype=np.float64)
        self.dist_coeffs = None
        
        # Set up ArUco detector
        self.dictionary = cv2.aruco.Dictionary_get(self.dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # Set up ROS publishers/subscribers
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
        self.markers_pub = rospy.Publisher('~markers', ArucoMarkerArray, queue_size=1)
        self.vis_markers_pub = rospy.Publisher('~visualization', MarkerArray, queue_size=1)
        
        # Subscribers
        # For compressed images
        image_topic = rospy.get_param('~image_topic', '/raspicam_node/image/compressed')
        if 'compressed' in image_topic:
            rospy.Subscriber(image_topic, CompressedImage, self.compressed_image_callback, queue_size=1)
        else:
            rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
            
        rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber('map_markers', ArucoMarkerArray, self.map_markers_callback, queue_size=1)
        
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Service
        self.set_markers_srv = rospy.Service('~set_length_override', SetMarkers, self.set_markers)
        
        # Dynamic reconfigure
        self.dyn_srv = Server(DetectorConfig, self.param_callback)
        
        self.map_markers_ids = set()
        self.vis_array = MarkerArray()
        
        rospy.loginfo("ArUco detector ready")
    
    def read_length_override(self):
        """Read marker length overrides from parameters"""
        length_override_dict = rospy.get_param('~length_override', {})
        for key, value in length_override_dict.items():
            self.length_override[int(key)] = value
    
    def get_marker_length(self, marker_id):
        """Get marker length, with override if available"""
        if marker_id in self.length_override:
            return self.length_override[marker_id]
        return self.length
    
    def camera_info_callback(self, msg):
        """Process camera calibration data"""
        self.camera_matrix = np.reshape(msg.K, (3, 3))
        self.dist_coeffs = np.array(msg.D)
    
    def get_child_frame_id(self, marker_id):
        """Generate child frame ID for a marker"""
        return self.frame_id_prefix + str(marker_id)
    
    def set_markers(self, req):
        """Service handler for setting marker length overrides"""
        res = SetMarkersResponse()
        
        for marker in req.markers:
            if marker.id > 999:
                res.message = f"Invalid marker id: {marker.id}"
                rospy.logerr(res.message)
                return res
            
            if not math.isfinite(marker.length) or marker.length <= 0:
                res.message = f"Invalid marker {marker.id} length: {marker.length}"
                rospy.logerr(res.message)
                return res
        
        for marker in req.markers:
            self.length_override[marker.id] = marker.length
        
        res.success = True
        return res
    
    def map_markers_callback(self, msg):
        """Process markers from map"""
        self.map_markers_ids.clear()
        
        for marker in msg.markers:
            self.map_markers_ids.add(marker.id)
            if self.use_map_markers:
                if marker.id not in self.length_override:
                    self.length_override[marker.id] = marker.length
        
        self.waiting_for_map = False
    
    def push_vis_markers(self, frame_id, stamp, pose, length, marker_id, index):
        """Create visualization markers for detected ArUco markers"""
        # Marker cube
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.ADD
        marker.id = index
        marker.ns = "aruco_marker"
        marker.type = Marker.CUBE
        marker.scale.x = length
        marker.scale.y = length
        marker.scale.z = 0.001
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.pose = pose
        self.vis_array.markers.append(marker)
        
        # Label
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.ADD
        marker.id = index
        marker.ns = "aruco_marker_label"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = length * 0.6
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = str(marker_id)
        marker.pose = pose
        self.vis_array.markers.append(marker)
    
    def process_image(self, image, header):
        """Process image to detect markers"""
        if not self.enabled or self.waiting_for_map:
            return
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
        
        # Create marker array message
        array = ArucoMarkerArray()
        array.header = header
        
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            
            # Estimate poses if needed
            rvecs = None
            tvecs = None
            
            if self.estimate_poses:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.length, 
                                                                 self.camera_matrix, self.dist_coeffs)
                
                # Process length override
                if self.length_override:
                    for i, marker_id in enumerate(ids):
                        if marker_id in self.length_override:
                            # Re-estimate with correct length
                            corners_current = [corners[i]]
                            rvecs_current, tvecs_current = cv2.aruco.estimatePoseSingleMarkers(
                                corners_current, self.length_override[marker_id],
                                self.camera_matrix, self.dist_coeffs
                            )
                            rvecs[i] = rvecs_current[0]
                            tvecs[i] = tvecs_current[0]
                
                # Get known vertical if needed
                vertical = None
                if self.known_vertical:
                    try:
                        vertical = self.tf_buffer.lookup_transform(
                            header.frame_id, self.known_vertical,
                            header.stamp, self.transform_timeout
                        )
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        rospy.logwarn_throttle(5, f"Can't retrieve known vertical: {e}")
            
            # Process each detected marker
            transforms = []
            
            for i, marker_id in enumerate(ids):
                marker = ArucoMarker()
                marker.id = int(marker_id)
                marker.length = self.get_marker_length(marker_id)
                
                # Fill corner points
                fill_corners(marker, corners[i][0])
                
                if self.estimate_poses:
                    # Fill pose data
                    fill_pose(marker.pose, rvecs[i], tvecs[i])
                    
                    # Apply known vertical if available
                    if self.known_vertical and vertical is not None:
                        apply_vertical(marker.pose.orientation, vertical.transform.rotation, 
                                      self.flip_vertical, self.auto_flip)
                    
                    # Create transform if needed
                    if self.send_tf:
                        transform = TransformStamped()
                        transform.header = header
                        transform.child_frame_id = self.get_child_frame_id(marker_id)
                        
                        # Check if marker is in the map
                        if marker_id not in self.map_markers_ids:
                            # Check if a marker with that id is already added
                            send = True
                            for t in transforms:
                                if t.child_frame_id == transform.child_frame_id:
                                    send = False
                                    break
                            
                            if send:
                                transform.transform.rotation = marker.pose.orientation
                                fill_translation(transform.transform.translation, tvecs[i])
                                transforms.append(transform)
                
                array.markers.append(marker)
            
            # Send transforms
            if self.send_tf and transforms:
                self.br.sendTransform(transforms)
        
        # Publish marker array
        self.markers_pub.publish(array)
        
        # Publish visualization markers
        if self.estimate_poses and self.vis_markers_pub.get_num_connections() > 0:
            # Delete all markers
            vis_marker = Marker()
            vis_marker.action = Marker.DELETEALL
            self.vis_array = MarkerArray()
            self.vis_array.markers.append(vis_marker)
            
            if ids is not None:
                for i, marker_id in enumerate(ids):
                    self.push_vis_markers(
                        header.frame_id, header.stamp,
                        array.markers[i].pose, self.get_marker_length(marker_id),
                        marker_id, i
                    )
            
            self.vis_markers_pub.publish(self.vis_array)
        
        # Publish debug image
        if self.debug_pub.get_num_connections() > 0:
            debug_img = image.copy()
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
                
                if self.estimate_poses:
                    for i, marker_id in enumerate(ids):
                        cv2.aruco.drawAxis(debug_img, self.camera_matrix, self.dist_coeffs,
                                         rvecs[i], tvecs[i], self.get_marker_length(marker_id))
            
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
    
    def compressed_image_callback(self, msg):
        """Process compressed image"""
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.process_image(image, msg.header)
    
    def image_callback(self, msg):
        """Process raw image"""
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_image(image, msg.header)
    
    def param_callback(self, config, level):
        """Dynamic reconfigure callback"""
        self.enabled = config.enabled and config.length > 0
        self.length = config.length
        
        # Update detector parameters
        self.parameters.adaptiveThreshConstant = config.adaptiveThreshConstant
        self.parameters.adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin
        self.parameters.adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax
        self.parameters.adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep
        self.parameters.cornerRefinementMaxIterations = config.cornerRefinementMaxIterations
        self.parameters.cornerRefinementMethod = config.cornerRefinementMethod
        self.parameters.cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy
        self.parameters.cornerRefinementWinSize = config.cornerRefinementWinSize
        
        # OpenCV version dependent parameters
        cv_version = cv2.__version__.split('.')
        major = int(cv_version[0])
        minor = int(cv_version[1]) if len(cv_version) > 1 else 0
        
        if (major == 3 and minor >= 4) or major > 3:
            self.parameters.detectInvertedMarker = config.detectInvertedMarker
        
        self.parameters.errorCorrectionRate = config.errorCorrectionRate
        self.parameters.minCornerDistanceRate = config.minCornerDistanceRate
        self.parameters.markerBorderBits = config.markerBorderBits
        self.parameters.maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate
        self.parameters.minDistanceToBorder = config.minDistanceToBorder
        self.parameters.minMarkerDistanceRate = config.minMarkerDistanceRate
        self.parameters.minMarkerPerimeterRate = config.minMarkerPerimeterRate
        self.parameters.maxMarkerPerimeterRate = config.maxMarkerPerimeterRate
        self.parameters.minOtsuStdDev = config.minOtsuStdDev
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell
        self.parameters.perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell
        self.parameters.polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate
        
        if (major == 3 and minor >= 4) or major > 3:
            self.parameters.aprilTagQuadDecimate = config.aprilTagQuadDecimate
            self.parameters.aprilTagQuadSigma = config.aprilTagQuadSigma
        
        return config


if __name__ == '__main__':
    try:
        detector = ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 