#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Drawing functions for ArUco markers
Based on the C++ implementation from draw.cpp and draw.h
"""

import math
import numpy as np
import cv2

def draw_planar_board(board, out_size, margin_size=0, border_bits=1, draw_axis=False):
    """
    Draw a planar ArUco board
    
    Args:
        board: ArUco board object with dictionary, ids, and objPoints
        out_size: Output image size (width, height)
        margin_size: Margin size in pixels
        border_bits: Width of marker borders
        draw_axis: Whether to draw coordinate axes
    
    Returns:
        Image with the board
    """
    # Create output image
    if draw_axis:
        img = np.ones((out_size[1], out_size[0], 3), dtype=np.uint8) * 255
    else:
        img = np.ones((out_size[1], out_size[0]), dtype=np.uint8) * 255
    
    # Apply margins
    if margin_size > 0:
        roi = img[margin_size:-margin_size, margin_size:-margin_size]
    else:
        roi = img
    
    # Calculate min and max values in XY plane
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    
    for obj_points in board.objPoints:
        for point in obj_points:
            min_x = min(min_x, point[0])
            max_x = max(max_x, point[0])
            min_y = min(min_y, point[1])
            max_y = max(max_y, point[1])
    
    size_x = max_x - min_x
    size_y = max_y - min_y
    
    # Proportion transformations
    x_reduction = size_x / roi.shape[1]
    y_reduction = size_y / roi.shape[0]
    
    # Determine the zone where markers are placed
    if x_reduction > y_reduction:
        n_rows = int(size_y / x_reduction)
        rows_margins = (roi.shape[0] - n_rows) // 2
        if rows_margins > 0:
            roi = roi[rows_margins:-rows_margins, :]
    else:
        n_cols = int(size_x / y_reduction)
        cols_margins = (roi.shape[1] - n_cols) // 2
        if cols_margins > 0:
            roi = roi[:, cols_margins:-cols_margins]
    
    # Paint each marker
    dictionary = board.dictionary
    
    for m in range(len(board.ids)):
        # Transform corners to markerZone coordinates
        out_corners = []
        for j in range(3):  # We only need 3 corners for the affine transform
            p = board.objPoints[m][j]
            # Move top left to 0, 0
            p = np.array([p[0] - min_x, p[1] - min_y])
            p[0] = p[0] / size_x * roi.shape[1]
            p[1] = (1.0 - p[1] / size_y) * roi.shape[0]
            out_corners.append(p)
        
        out_corners = np.array(out_corners, dtype=np.float32)
        
        # Get marker
        dst_sz = np.linalg.norm(out_corners[2] - out_corners[0])  # assuming CCW order
        side = max(int(round(dst_sz / math.sqrt(2))), 10)
        
        marker_img = dictionary.drawMarker(board.ids[m], side, borderBits=border_bits)
        if draw_axis:
            marker_img = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2RGB)
        
        # Interpolate tiny marker to marker position in markerZone
        in_corners = np.array([
            [0, 0],
            [marker_img.shape[1], 0],
            [marker_img.shape[1], marker_img.shape[0]]
        ], dtype=np.float32)
        
        # Remove perspective
        transformation = cv2.getAffineTransform(in_corners, out_corners)
        cv2.warpAffine(marker_img, transformation, (roi.shape[1], roi.shape[0]), 
                      dst=roi, borderMode=cv2.BORDER_TRANSPARENT)
    
    # Draw axis
    if draw_axis:
        # Find center point in the image
        center_x = margin_size + roi.shape[1] // 2
        center_y = img.shape[0] - margin_size - roi.shape[0] // 2
        center = (center_x, center_y)
        
        # Define axis points and colors
        axis_points = [(300, 0), (0, -300), (-150, 150)]
        axis_names = [(270, 50), (25, -270), (-160, 115)]
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # RGB for X, Y, Z
        names = ["X", "Y", "Z"]
        
        r_half = 14
        height = 55
        
        for i in range(2, -1, -1):
            # Calculate arrow angles
            alpha = math.atan2(0 - axis_points[i][0], 0 - axis_points[i][1])
            x_delta = r_half * math.cos(alpha)
            y_delta = r_half * math.sin(alpha)
            
            # Create arrow polygon vertices
            polygon = np.array([
                [center[0] + axis_points[i][0] + x_delta, center[1] + axis_points[i][1] - y_delta],
                [center[0] + axis_points[i][0] - x_delta, center[1] + axis_points[i][1] + y_delta],
                [center[0] + axis_points[i][0] - math.sin(alpha) * height, 
                 center[1] + axis_points[i][1] - math.cos(alpha) * height]
            ], dtype=np.int32)
            
            # Draw arrow
            cv2.fillPoly(img, [polygon], colors[i])
            
            # Draw axis label
            cv2.putText(img, names[i], 
                       (center[0] + axis_names[i][0], center[1] + axis_names[i][1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, colors[i], 7)
            
            # Draw axis line
            cv2.line(img, center, 
                    (center[0] + axis_points[i][0], center[1] + axis_points[i][1]), 
                    colors[i], 10)
    
    return img

def draw_axis(image, camera_matrix, dist_coeffs, rvec, tvec, length=1.0):
    """
    Draw 3D axes on an image
    
    Args:
        image: Input image
        camera_matrix: Camera matrix
        dist_coeffs: Distortion coefficients
        rvec: Rotation vector
        tvec: Translation vector
        length: Length of the axes
    """
    # Define axis points in 3D space
    axis_points = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
    
    # Project 3D points to image plane
    img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_points = img_points.reshape(-1, 2)
    
    # Draw axes
    origin = tuple(img_points[0].astype(int))
    cv2.line(image, origin, tuple(img_points[1].astype(int)), (0, 0, 255), 3)  # X-axis (red)
    cv2.line(image, origin, tuple(img_points[2].astype(int)), (0, 255, 0), 3)  # Y-axis (green)
    cv2.line(image, origin, tuple(img_points[3].astype(int)), (255, 0, 0), 3)  # Z-axis (blue) 