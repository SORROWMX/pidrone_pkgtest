#!/usr/bin/env python

import rospy
import time
import struct
import numpy as np
import cv2
from geometry_msgs.msg import TwistStamped
from raspicam_node.msg import MotionVectors
from sensor_msgs.msg import Range, CompressedImage
from unavlib.control.uavcontrol import UAVControl
from unavlib import MSPy

# Format for MSP2_SENSOR_OPTIC_FLOW data
# <Bff: < (little endian), B (unsigned char - quality), f (float - motionX), f (float - motionY)
MSP2_FLOW_FORMAT = '<Bff'

# MSP codes for optic flow data
MSP2_SENSOR_OPTIC_FLOW = 0x1F02  # 7938 decimal - correct code for optical flow

class OpticalFlowRelay:
    """
    Subscribes to the camera images, processes them to detect optical flow using feature detection,
    and sends the flow data to the flight controller using the MSP2_SENSOR_OPTIC_FLOW command for position hold.
    
    This implementation uses OpenCV feature detection and matching for more robust flow estimation.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('optical_flow_relay')
        
        # Flight controller connection parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.update_rate = rospy.get_param('~update_rate', 40)  # Hz - 40Hz for good performance
        
        # Connect to flight controller
        self.board = None
        self.connect_to_fc()
        
        # Camera parameters
        self.camera_wh = (320, 240)  # Camera resolution
        
        # PIXEL SIZE of Pi camera v2
        self.PIXEL_SIZE_M = 1.12 * 10e-6  # 1.12um for Pi Camera v2
        
        # Focal Length of Pi camera v2
        self.FOCAL_LENGTH_M = 3.04 * 10e-3  # 3.04mm for Pi Camera v2
        
        # Calculate focal length in pixels
        self.FOCAL_LENGTH_PX = self.FOCAL_LENGTH_M / self.PIXEL_SIZE_M
        
        # Flow variables
        self.flow_scale = self.PIXEL_SIZE_M / self.FOCAL_LENGTH_M  # Angular conversion
        
        # Last received flow data
        self.last_motion_x = 0.0
        self.last_motion_y = 0.0
        self.last_quality = 255  # Default quality (0-255)
        self.altitude = 0.03  # initialize to a bit off the ground
        self.last_update_time = rospy.Time.now()
        
        # OpenCV feature detection
        self.prev_image = None
        self.current_image = None
        self.orb = cv2.ORB_create(50)  # Use ORB with max 50 features
        
        # Subscribe to camera image, motion vectors and altitude
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        rospy.Subscriber('/pidrone/range', Range, self.altitude_callback, queue_size=1)
        
        # Timer for sending data to flight controller
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_flow_data)
        
        rospy.loginfo("Advanced Optical Flow relay initialized")
        
        # Print board capabilities
        self.print_board_info()
        
        # Tracking successful sends
        self.send_count = 0
        self.last_send_time = rospy.Time.now()
        self.altitude_ts = rospy.Time.now()

    def print_board_info(self):
        """Print board information for debugging"""
        try:
            # Get board info
            if hasattr(self.board.board, 'SENSOR_DATA'):
                rospy.loginfo(f"SENSOR_DATA available: {self.board.board.SENSOR_DATA}")
            else:
                rospy.loginfo("SENSOR_DATA not available")
                
            # Print available MSP codes
            if hasattr(MSPy, 'MSPCodes'):
                rospy.loginfo("MSPy.MSPCodes is available")
                # Check if optic flow code exists
                if 'MSP2_SENSOR_OPTIC_FLOW' in MSPy.MSPCodes:
                    rospy.loginfo(f"MSP2_SENSOR_OPTIC_FLOW code: {MSPy.MSPCodes['MSP2_SENSOR_OPTIC_FLOW']}")
                else:
                    rospy.loginfo("MSP2_SENSOR_OPTIC_FLOW code not found in MSPy.MSPCodes")
            else:
                rospy.loginfo("MSPy.MSPCodes not available")
        except Exception as e:
            rospy.logwarn(f"Could not print board info: {e}")

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

    def altitude_callback(self, msg):
        """Process incoming altitude data"""
        self.altitude = msg.range
        self.altitude_ts = rospy.Time.now()

    def image_callback(self, msg):
        """Process incoming camera images and calculate optical flow"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.fromstring(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            
            # Store first image
            if self.prev_image is None:
                self.prev_image = image
                return
                
            # Calculate time delta since last update
            now = rospy.Time.now()
            dt = (now - self.last_update_time).to_sec()
            self.last_update_time = now
            
            # Skip if dt is too large (first reading or long delay)
            if dt > 0.1:
                dt = 0.025  # Use default value for first reading or long delays
                
            # Detect features in both images
            kp1, des1 = self.orb.detectAndCompute(self.prev_image, None)
            kp2, des2 = self.orb.detectAndCompute(image, None)
            
            # If we have features in both images
            if kp1 and kp2 and des1 is not None and des2 is not None:
                # Match features using Brute Force matcher
                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                matches = bf.match(des1, des2)
                
                # Sort matches by distance (lower is better)
                matches = sorted(matches, key=lambda x: x.distance)
                
                # Filter matches by distance threshold
                good_matches = [m for m in matches if m.distance < 38]
                
                if len(good_matches) >= 3:
                    # Extract matched keypoints
                    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    
                    # Calculate rigid transformation
                    if len(good_matches) >= 3:
                        # Calculate average displacement
                        displacement = np.mean(src_pts.reshape(-1, 2) - dst_pts.reshape(-1, 2), axis=0)
                        
                        # Convert to angular displacement in radians
                        # Using small angle approximation: angle ≈ displacement/focal_length
                        self.last_motion_x = float(displacement[0] / self.FOCAL_LENGTH_PX)
                        self.last_motion_y = float(displacement[1] / self.FOCAL_LENGTH_PX)
                        
                        # Set quality based on number of good matches
                        match_quality = min(255, int(len(good_matches) * 5))
                        self.last_quality = match_quality
                    else:
                        self.last_motion_x = 0.0
                        self.last_motion_y = 0.0
                        self.last_quality = 0
                else:
                    # Not enough good matches
                    self.last_motion_x = 0.0
                    self.last_motion_y = 0.0
                    self.last_quality = 0
            else:
                # No features detected
                self.last_motion_x = 0.0
                self.last_motion_y = 0.0
                self.last_quality = 0
            
            # Log flow data occasionally
            if (now - self.last_send_time).to_sec() >= 1.0:  # Log once per second
                rospy.loginfo(f"Flow: X={self.last_motion_x:.6f}, Y={self.last_motion_y:.6f}, quality={self.last_quality}, altitude={self.altitude:.3f}m")
                self.last_send_time = now
            
            # Check if altitude data is recent
            duration_from_last_altitude = now - self.altitude_ts
            if duration_from_last_altitude.to_sec() > 10:
                rospy.logwarn(f"No altitude received for {duration_from_last_altitude.to_sec():.1f} seconds.")
            
            # Update previous image
            self.prev_image = image
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def send_flow_data(self, event=None):
        """Send optical flow data to flight controller"""
        if self.board is None:
            return
            
        try:
            # Prepare flow data message
            flow_data = {
                'quality': self.last_quality,
                'motionX': self.last_motion_x,
                'motionY': self.last_motion_y
            }
            
            # Pack data according to MSP protocol - using float format
            packed_data = struct.pack(MSP2_FLOW_FORMAT, 
                                      flow_data['quality'], 
                                      flow_data['motionX'], 
                                      flow_data['motionY'])
            
            # Use direct board access for sending MSP commands
            msp_code = MSP2_SENSOR_OPTIC_FLOW
            
            # Send using the direct board object
            result = self.board.board.send_RAW_msg(msp_code, data=packed_data)
            
            if result > 0:
                # For MSP2_SENSOR_OPTIC_FLOW, we don't expect a response
                # The flight controller just receives the data
                self.send_count += 1
                
                # Log success message periodically to avoid flooding logs
                if self.send_count % 40 == 0:  # Log every 40 successful sends (approximately once per second)
                    rospy.loginfo(f"Successfully sent {self.send_count} flow updates. Latest: X={flow_data['motionX']:.6f}, Y={flow_data['motionY']:.6f}, Q={flow_data['quality']}")
            else:
                rospy.logwarn("Failed to send optical flow data")
            
        except Exception as e:
            rospy.logerr(f"Error sending optical flow data: {e}")
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
    relay = OpticalFlowRelay()
    
    # Register shutdown hook
    rospy.on_shutdown(relay.shutdown)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 