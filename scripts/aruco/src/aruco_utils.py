#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Utility functions for ArUco marker detection and pose estimation
Based on the C++ implementation from utils.h
"""

import math
import numpy as np
import tf2_ros
import tf.transformations
from geometry_msgs.msg import Quaternion, Vector3, Transform, Pose

def parse_camera_info(camera_info):
    """Parse camera info message to get camera matrix and distortion coefficients"""
    camera_matrix = np.zeros((3, 3), dtype=np.float64)
    for i in range(3):
        for j in range(3):
            camera_matrix[i, j] = camera_info.K[3 * i + j]
    dist_coeffs = np.array(camera_info.D, dtype=np.float64)
    return camera_matrix, dist_coeffs

def rotate_point(p, origin, angle):
    """Rotate a point around an origin by an angle"""
    s = math.sin(angle)
    c = math.cos(angle)
    
    # Translate point to origin
    p_x = p.x - origin.x
    p_y = p.y - origin.y
    
    # Rotate point
    x_new = p_x * c - p_y * s
    y_new = p_x * s + p_y * c
    
    # Translate point back
    p.x = x_new + origin.x
    p.y = y_new + origin.y

def fill_pose(pose, rvec, tvec):
    """Fill a ROS Pose message from rotation and translation vectors"""
    pose.position.x = tvec[0]
    pose.position.y = tvec[1]
    pose.position.z = tvec[2]
    
    angle = np.linalg.norm(rvec)
    if angle < 1e-10:  # Avoid division by zero
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        return
        
    axis = rvec / angle
    
    # Convert axis-angle to quaternion
    q = tf.transformations.quaternion_about_axis(angle, axis)
    
    pose.orientation.w = q[3]  # TF quaternion format is [x, y, z, w]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]

def fill_transform(transform, rvec, tvec):
    """Fill a ROS Transform message from rotation and translation vectors"""
    transform.translation.x = tvec[0]
    transform.translation.y = tvec[1]
    transform.translation.z = tvec[2]
    
    angle = np.linalg.norm(rvec)
    if angle < 1e-10:  # Avoid division by zero
        transform.rotation.w = 1.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        return
        
    axis = rvec / angle
    
    # Convert axis-angle to quaternion
    q = tf.transformations.quaternion_about_axis(angle, axis)
    
    transform.rotation.w = q[3]  # TF quaternion format is [x, y, z, w]
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]

def fill_translation(translation, tvec):
    """Fill a ROS Vector3 message from a translation vector"""
    translation.x = tvec[0]
    translation.y = tvec[1]
    translation.z = tvec[2]

def is_flipped(q):
    """Check if orientation is flipped"""
    # Convert quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    pitch = euler[1]
    roll = euler[0]
    return abs(pitch) > math.pi / 2 or abs(roll) > math.pi / 2

def apply_vertical(orientation, vertical, flip_vertical=False, auto_flip=False):
    """Apply a vertical orientation to another orientation"""
    # Convert ROS quaternion messages to TF quaternions
    q_vertical = [vertical.x, vertical.y, vertical.z, vertical.w]
    q_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
    
    # Check if we need to flip the vertical
    if flip_vertical or (auto_flip and not is_flipped(orientation)):
        # Create flip quaternion (180 degrees around X axis)
        q_flip = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
        # Apply flip to vertical
        q_vertical = tf.transformations.quaternion_multiply(q_vertical, q_flip)
    
    # Get rotation matrix from orientation and vertical
    mat_orientation = tf.transformations.quaternion_matrix(q_orientation)
    mat_vertical = tf.transformations.quaternion_matrix(q_vertical)
    
    # Calculate difference in yaw
    # Extract yaw from the difference between orientation and vertical
    mat_diff = np.dot(np.transpose(mat_orientation[:3, :3]), mat_vertical[:3, :3])
    euler = tf.transformations.euler_from_matrix(mat_diff)
    yaw = euler[2]  # Extract yaw
    
    # Create quaternion for yaw correction
    q_yaw = tf.transformations.quaternion_from_euler(0, 0, -yaw)
    
    # Apply yaw from orientation to vertical
    q_result = tf.transformations.quaternion_multiply(q_vertical, q_yaw)
    
    # Set result back to orientation
    orientation.w = q_result[3]
    orientation.x = q_result[0]
    orientation.y = q_result[1]
    orientation.z = q_result[2]

def transform_to_pose(transform, pose):
    """Convert a Transform message to a Pose message"""
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation = transform.rotation

def fill_corners(marker, corners):
    """Fill marker corners from OpenCV detection"""
    marker.c1.x = corners[0][0]
    marker.c2.x = corners[1][0]
    marker.c3.x = corners[2][0]
    marker.c4.x = corners[3][0]
    marker.c1.y = corners[0][1]
    marker.c2.y = corners[1][1]
    marker.c3.y = corners[2][1]
    marker.c4.y = corners[3][1] 