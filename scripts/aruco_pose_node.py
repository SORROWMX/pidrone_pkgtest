#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ArucoPoseNode:
    def __init__(self):
        rospy.init_node('aruco_pose_node', anonymous=True)

        # --- Parameters ---
        # Camera calibration
        # FIXME: You need to provide your camera calibration parameters from a .yaml file
        # You can get these by running the camera_calibration package:
        # rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/raspicam_node/image_raw
        # For now, using placeholder values.
        default_camera_matrix = [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0]
        camera_matrix_param = rospy.get_param('~camera_matrix', default_camera_matrix)
        self.camera_matrix = np.array(camera_matrix_param).reshape(3, 3)
        
        distortion_param = rospy.get_param('~distortion_coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.dist_coeffs = np.array(distortion_param)

        # ArUco parameters
        self.marker_length = rospy.get_param('~marker_length', 0.1) # Default marker length in meters
        dictionary_name = rospy.get_param('~dictionary', 'DICT_4X4_50')
        self.aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dictionary_name))
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # --- ROS ---
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        
        # Publishers
        self.debug_pub = rospy.Publisher('~debug_image/compressed', CompressedImage, queue_size=1)
        
        rospy.loginfo("Aruco pose node started.")
        rospy.loginfo("Waiting for images on topic /raspicam_node/image/compressed...")
        rospy.loginfo(f"Using ArUco dictionary: {dictionary_name}")
        rospy.loginfo(f"Default marker length: {self.marker_length}m")
        rospy.loginfo("Please provide camera calibration parameters for accurate pose estimation.")


    def image_callback(self, msg):
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Failed to decompress image: {e}")
            return

        if cv_image is None:
            rospy.logwarn("Decoded image is empty.")
            return
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        debug_image = cv_image.copy()

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
            
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                # Draw axis for each marker
                cv2.aruco.drawAxis(debug_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)

        # Publish debug image
        if self.debug_pub.get_num_connections() > 0:
            msg_out = self.bridge.cv2_to_compressed_imgmsg(debug_image)
            msg_out.header = msg.header
            self.debug_pub.publish(msg_out)


if __name__ == '__main__':
    try:
        node = ArucoPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 