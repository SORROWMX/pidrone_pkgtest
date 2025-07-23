#!/usr/bin/env python3

import rospy
import time
import struct
import numpy as np
import cv2
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from unavlib.control.uavcontrol import UAVControl
from unavlib import MSPy
import tf2_ros
import tf.transformations

# Format for MSP2_SENSOR_OPTIC_FLOW data (INAV)
# <Bff: < (little endian), B (unsigned char - quality), f (float - motionX), f (float - motionY)
MSP2_FLOW_FORMAT = '<Bff'

# MSP codes for optic flow data
MSP2_SENSOR_OPTIC_FLOW = 0x1F02  # 7938 decimal - correct code for optical flow

class OpticalFlowINAV:
    """
    Advanced Optical Flow for INAV using phase correlation algorithm from OpenCV.
    This combines the best parts of optical_flow.cpp (phase correlation) and 
    test_msp2_optical_flow.py (MSP protocol for INAV).
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('optical_flow_inav')
        
        # Parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.update_rate = rospy.get_param('~update_rate', 40)  # Hz
        self.debug = rospy.get_param('~debug', False)
        self.roi_px = rospy.get_param('~roi', 128)  # Region of interest in pixels
        self.roi_rad = rospy.get_param('~roi_rad', 0.0)  # Region of interest in radians
        self.fcu_frame_id = rospy.get_param('~fcu_frame_id', 'base_link')
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')
        
        # Connect to flight controller
        self.board = None
        self.connect_to_fc()
        
        # Initialize OpenCV objects
        self.bridge = CvBridge()
        self.prev_image = None
        self.curr_image = None
        self.hann = None  # Hanning window for phase correlation
        self.prev_stamp = None
        self.camera_matrix = np.zeros((3, 3), dtype=np.float64)
        self.dist_coeffs = None
        self.roi = None
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Flow data
        self.last_quality = 255
        self.last_motion_x = 0.0
        self.last_motion_y = 0.0
        
        # Publishers
        self.debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
        self.velo_pub = rospy.Publisher('~angular_velocity', TwistStamped, queue_size=1)
        self.shift_pub = rospy.Publisher('~shift', Vector3Stamped, queue_size=1)
        
        # Subscribers
        self.img_sub = rospy.Subscriber('/main_camera/image_raw', Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/main_camera/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        
        # Timer for sending data to flight controller
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_flow_data)
        
        # Stats
        self.send_count = 0
        self.last_send_time = rospy.Time.now()
        
        rospy.loginfo("Optical Flow for INAV initialized")
    
    def connect_to_fc(self):
        """Connect to the flight controller board"""
        try:
            rospy.loginfo(f"Connecting to flight controller on {self.serial_port}")
            self.board = UAVControl(self.serial_port, self.baudrate, receiver="serial")
            self.board.connect()
            rospy.loginfo("Connected to flight controller")
        except Exception as e:
            rospy.logerr(f"Failed to connect to flight controller: {e}")
            try:
                # Try alternative port
                alt_port = '/dev/ttyACM1'
                rospy.loginfo(f"Trying alternative port {alt_port}")
                self.board = UAVControl(alt_port, self.baudrate, receiver="serial")
                self.board.connect()
                rospy.loginfo("Connected to flight controller on alternative port")
            except Exception as e:
                rospy.logerr(f"Failed to connect to flight controller on alternative port: {e}")
                rospy.signal_shutdown("Could not connect to flight controller")
    
    def camera_info_callback(self, msg):
        """Process camera calibration data"""
        # Extract camera matrix
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        
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
                
                rospy.loginfo(f"ROI: {self.roi.x} {self.roi.y} - {self.roi.x + self.roi.width} {self.roi.y + self.roi.height}")
            else:
                # Use fixed size ROI centered in the image
                center_x = msg.width // 2
                center_y = msg.height // 2
                half_roi = self.roi_px // 2
                self.roi = cv2.Rect(center_x - half_roi, center_y - half_roi, self.roi_px, self.roi_px)
    
    def image_callback(self, msg):
        """Process incoming raw camera images and calculate optical flow"""
        try:
            # Convert ROS Image message to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            # Apply ROI if defined
            if self.roi is not None:
                # Make sure ROI is within image bounds
                valid_roi = cv2.Rect(0, 0, img.shape[1], img.shape[0]) & self.roi
                if valid_roi.width > 0 and valid_roi.height > 0:
                    img = img[valid_roi.y:valid_roi.y+valid_roi.height, valid_roi.x:valid_roi.x+valid_roi.width]
            
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
            
            # Undistort the point
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
            
            # Publish debug image
            if self.debug_pub.get_num_subscribers() > 0:
                # Draw flow vector
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
            
            # Log flow data occasionally in debug mode
            if self.debug and (rospy.Time.now() - self.last_send_time).to_sec() >= 1.0:
                rospy.loginfo(f"Flow: X={self.last_motion_x:.6f}, Y={self.last_motion_y:.6f}, quality={self.last_quality}")
                self.last_send_time = rospy.Time.now()
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
            if self.debug:
                import traceback
                rospy.logerr(traceback.format_exc())
    
    def send_flow_data(self, event=None):
        """Send optical flow data to flight controller using MSP protocol"""
        if self.board is None:
            return
            
        try:
            # Pack data according to MSP protocol
            packed_data = struct.pack(
                MSP2_FLOW_FORMAT, 
                self.last_quality,
                self.last_motion_x,
                self.last_motion_y
            )
            
            # Send using the direct board object
            result = self.board.board.send_RAW_msg(MSP2_SENSOR_OPTIC_FLOW, data=packed_data)
            
            if result > 0:
                self.send_count += 1
                
                # Log success message periodically in debug mode
                if self.debug and self.send_count % 40 == 0:
                    rospy.loginfo(f"Successfully sent {self.send_count} flow updates. Latest: X={self.last_motion_x:.6f}, Y={self.last_motion_y:.6f}, Q={self.last_quality}")
            else:
                # Log failures
                rospy.logwarn("Failed to send optical flow data")
                
        except Exception as e:
            rospy.logerr(f"Error sending optical flow data: {e}")
            if self.debug:
                import traceback
                rospy.logerr(traceback.format_exc())
    
    def shutdown(self):
        """Clean up when node is shutting down"""
        if self.board:
            try:
                self.board.disconnect()
                rospy.loginfo(f"Disconnected from flight controller after sending {self.send_count} flow updates")
            except:
                pass

def main():
    flow = OpticalFlowINAV()
    
    # Register shutdown hook
    rospy.on_shutdown(flow.shutdown)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 