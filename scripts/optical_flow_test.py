#!/usr/bin/env python3

import rospy
import time
import numpy as np
import cv2
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import tf.transformations

class OpticalFlowTest:
    """
    Test version of Optical Flow algorithm without flight controller connection.
    This is for testing and visualization only.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('optical_flow_test')
        
        # Parameters
        self.debug = rospy.get_param('~debug', True)  # Debug mode enabled by default for testing
        self.roi_px = rospy.get_param('~roi', 128)  # Region of interest in pixels
        self.roi_rad = rospy.get_param('~roi_rad', 0.0)  # Region of interest in radians
        self.fcu_frame_id = rospy.get_param('~fcu_frame_id', 'base_link')
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')
        self.image_topic = rospy.get_param('~image_topic', '/main_camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/main_camera/camera_info')
        
        # Initialize OpenCV objects
        self.bridge = CvBridge()
        self.prev_image = None
        self.curr_image = None
        self.hann = None  # Hanning window for phase correlation
        self.prev_stamp = None
        self.camera_matrix = np.zeros((3, 3), dtype=np.float64)
        self.dist_coeffs = None
        self.roi = None
        
        # Flow data
        self.last_quality = 255
        self.last_motion_x = 0.0
        self.last_motion_y = 0.0
        
        # Publishers
        self.debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
        self.velo_pub = rospy.Publisher('~angular_velocity', TwistStamped, queue_size=1)
        self.shift_pub = rospy.Publisher('~shift', Vector3Stamped, queue_size=1)
        self.flow_vis_pub = rospy.Publisher('~flow_visualization', Image, queue_size=1)
        
        # Subscribers
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        
        # Stats
        self.frame_count = 0
        self.last_log_time = rospy.Time.now()
        
        rospy.loginfo(f"Optical Flow Test initialized. Listening to {self.image_topic}")
    
    def camera_info_callback(self, msg):
        """Process camera calibration data"""
        # Extract camera matrix
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        
        rospy.loginfo(f"Received camera calibration. Matrix: {self.camera_matrix[0,0]:.1f}, {self.camera_matrix[1,1]:.1f}")
        
        # Calculate ROI if needed
        if self.roi is None and self.roi_px != 0:
            if self.roi_rad != 0:
                # Calculate ROI based on field of view in radians
                object_points = np.array([
                    [-np.sin(self.roi_rad / 2), -np.sin(self.roi_rad / 2), np.cos(self.roi_rad / 2)],
                    [np.sin(self.roi_rad / 2), np.sin(self.roi_rad / 2), np.cos(self.roi_rad / 2)]
                ])
                
                # Project 3D points to image plane
                img_points = cv2.projectPoints(
                    object_points, 
                    np.zeros(3), 
                    np.zeros(3), 
                    self.camera_matrix, 
                    self.dist_coeffs
                )[0].reshape(-1, 2)
                
                # Create ROI rectangle
                self.roi = cv2.Rect(
                    int(round(img_points[0][0])), 
                    int(round(img_points[0][1])), 
                    int(round(img_points[1][0] - img_points[0][0])), 
                    int(round(img_points[1][1] - img_points[0][1]))
                )
                
                rospy.loginfo(f"ROI (from radians): {self.roi.x} {self.roi.y} - {self.roi.x + self.roi.width} {self.roi.y + self.roi.height}")
            else:
                # Use fixed size ROI centered in the image
                center_x = msg.width // 2
                center_y = msg.height // 2
                half_roi = self.roi_px // 2
                self.roi = cv2.Rect(center_x - half_roi, center_y - half_roi, self.roi_px, self.roi_px)
                rospy.loginfo(f"ROI (fixed size): {self.roi.x} {self.roi.y} - {self.roi.x + self.roi.width} {self.roi.y + self.roi.height}")
    
    def image_callback(self, msg):
        """Process incoming raw camera images and calculate optical flow"""
        try:
            # Convert ROS Image message to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            # Store original image for visualization
            original_img = img.copy()
            
            # Apply ROI if defined
            roi_applied = False
            if self.roi is not None:
                # Make sure ROI is within image bounds
                valid_roi = cv2.Rect(0, 0, img.shape[1], img.shape[0]) & self.roi
                if valid_roi.width > 0 and valid_roi.height > 0:
                    img = img[valid_roi.y:valid_roi.y+valid_roi.height, valid_roi.x:valid_roi.x+valid_roi.width]
                    roi_applied = True
            
            # Convert to float for phase correlation
            self.curr_image = np.float32(img)
            
            # Initialize previous frame if needed
            if self.prev_image is None or (msg.header.stamp - self.prev_stamp).to_sec() > 0.1:
                self.prev_image = self.curr_image.copy()
                self.prev_stamp = msg.header.stamp
                # Create Hanning window for phase correlation
                self.hann = cv2.createHanningWindow(self.curr_image.shape, cv2.CV_32F)
                return
            
            # Calculate phase correlation
            response, shift = cv2.phaseCorrelate(self.prev_image, self.curr_image, self.hann)
            
            # Publish raw shift in pixels
            shift_vec = Vector3Stamped()
            shift_vec.header.stamp = msg.header.stamp
            shift_vec.header.frame_id = msg.header.frame_id
            shift_vec.vector.x = shift[0]
            shift_vec.vector.y = shift[1]
            self.shift_pub.publish(shift_vec)
            
            # Undistort flow in pixels
            flow_center_x = img.shape[1] // 2
            flow_center_y = img.shape[0] // 2
            
            # Adjust shift by center offset
            shift_point = np.array([[shift[0] + flow_center_x, shift[1] + flow_center_y]])
            
            # Undistort the point if camera matrix is available
            if np.sum(self.camera_matrix) > 0:
                undist_point = cv2.undistortPoints(
                    shift_point, 
                    self.camera_matrix, 
                    self.dist_coeffs, 
                    None, 
                    self.camera_matrix
                ).reshape(-1, 2)
                
                # Calculate flow in radians
                focal_length_x = self.camera_matrix[0, 0]
                focal_length_y = self.camera_matrix[1, 1]
                
                flow_x = np.arctan2(undist_point[0, 0] - flow_center_x, focal_length_x)
                flow_y = np.arctan2(undist_point[0, 1] - flow_center_y, focal_length_y)
            else:
                # Fallback if camera matrix is not available
                flow_x = shift[0] / 100.0  # Arbitrary scaling for testing
                flow_y = shift[1] / 100.0
            
            # Convert to FCU frame (camera Y axis -> FCU X axis, camera X axis -> FCU -Y axis)
            self.last_motion_x = flow_y
            self.last_motion_y = -flow_x
            self.last_quality = int(response * 255)
            
            # Calculate integration time
            integration_time = (msg.header.stamp - self.prev_stamp).to_sec()
            
            # Publish angular velocity
            velo = TwistStamped()
            velo.header.stamp = msg.header.stamp
            velo.header.frame_id = self.fcu_frame_id
            velo.twist.angular.x = self.last_motion_x / integration_time
            velo.twist.angular.y = self.last_motion_y / integration_time
            self.velo_pub.publish(velo)
            
            # Update previous image
            self.prev_image = self.curr_image.copy()
            self.prev_stamp = msg.header.stamp
            
            # Count frames
            self.frame_count += 1
            
            # Create and publish visualization
            if self.flow_vis_pub.get_num_subscribers() > 0:
                # Create color visualization
                vis_img = cv2.cvtColor(original_img, cv2.COLOR_GRAY2BGR)
                
                # Draw ROI if applied
                if roi_applied:
                    cv2.rectangle(
                        vis_img, 
                        (self.roi.x, self.roi.y), 
                        (self.roi.x + self.roi.width, self.roi.y + self.roi.height), 
                        (0, 255, 0), 
                        2
                    )
                
                # Draw flow vector on original image
                if roi_applied:
                    # Adjust center and endpoint for ROI
                    center_x = self.roi.x + flow_center_x
                    center_y = self.roi.y + flow_center_y
                    end_x = center_x + int(shift[0] * 5)
                    end_y = center_y + int(shift[1] * 5)
                else:
                    center_x = original_img.shape[1] // 2
                    center_y = original_img.shape[0] // 2
                    end_x = center_x + int(shift[0] * 5)
                    end_y = center_y + int(shift[1] * 5)
                
                # Draw flow vector
                cv2.arrowedLine(
                    vis_img,
                    (center_x, center_y),
                    (end_x, end_y),
                    (0, 0, 255),  # Red color
                    2
                )
                
                # Draw circle showing flow magnitude
                radius = int(np.sqrt(shift[0]**2 + shift[1]**2) * 5)
                cv2.circle(vis_img, (center_x, center_y), radius, (255, 0, 0), 1)
                
                # Add text with flow information
                cv2.putText(
                    vis_img,
                    f"Flow: {shift[0]:.2f}, {shift[1]:.2f} px",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )
                
                cv2.putText(
                    vis_img,
                    f"Rad: {self.last_motion_x:.4f}, {self.last_motion_y:.4f}",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )
                
                cv2.putText(
                    vis_img,
                    f"Quality: {self.last_quality}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )
                
                # Publish visualization
                vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
                vis_msg.header = msg.header
                self.flow_vis_pub.publish(vis_msg)
            
            # Also publish the debug image from the ROI with flow vector
            if self.debug_pub.get_num_subscribers() > 0:
                debug_img = img.copy()
                center = (debug_img.shape[1] // 2, debug_img.shape[0] // 2)
                cv2.line(
                    debug_img,
                    center,
                    (int(center[0] + shift[0] * 5), int(center[1] + shift[1] * 5)),
                    255,
                    2
                )
                
                # Create circle showing flow magnitude
                radius = int(np.sqrt(shift[0]**2 + shift[1]**2) * 5)
                cv2.circle(debug_img, center, radius, 255, 1)
                
                # Publish debug image
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='mono8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            
            # Log flow data occasionally
            if (rospy.Time.now() - self.last_log_time).to_sec() >= 1.0:
                fps = self.frame_count / (rospy.Time.now() - self.last_log_time).to_sec()
                rospy.loginfo(f"FPS: {fps:.1f}, Flow: X={self.last_motion_x:.6f}, Y={self.last_motion_y:.6f}, Q={self.last_quality}")
                self.last_log_time = rospy.Time.now()
                self.frame_count = 0
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

def main():
    flow = OpticalFlowTest()
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 