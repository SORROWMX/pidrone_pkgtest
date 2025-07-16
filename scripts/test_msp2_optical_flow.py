#!/usr/bin/env python

import rospy
import time
import struct
import numpy as np
from geometry_msgs.msg import TwistStamped
from raspicam_node.msg import MotionVectors
from sensor_msgs.msg import Range
from unavlib.control.uavcontrol import UAVControl
from unavlib import MSPy

# Format for MSP2_SENSOR_OPTIC_FLOW data
# <Bii: < (little endian), B (unsigned char - quality), i (signed int - motionX), i (signed int - motionY)
MSP2_FLOW_FORMAT = '<Bii'

# MSP codes for optic flow data
MSP2_SENSOR_OPTIC_FLOW = 0x1F02  # 7938 decimal - correct code for optical flow

class OpticalFlowRelay:
    """
    Subscribes to the optical flow vectors and sends them to the flight controller
    using the MSP2_SENSOR_OPTIC_FLOW command for position hold
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
        
        # Flow variables
        self.camera_wh = (320, 240)
        self.max_flow = self.camera_wh[0] / 16.0 * self.camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow
        
        # Last received flow data
        self.last_motion_x = 0
        self.last_motion_y = 0
        self.last_quality = 255  # Default quality (0-255)
        self.altitude = 0.03  # initialize to a bit off the ground
        
        # Subscribe to optical flow topic and altitude
        rospy.Subscriber('/raspicam_node/motion_vectors', MotionVectors, self.motion_vectors_callback, queue_size=1)
        rospy.Subscriber('/pidrone/range', Range, self.altitude_callback, queue_size=1)
        
        # Timer for sending data to flight controller
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_flow_data)
        
        rospy.loginfo("Optical Flow relay initialized")
        
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

    def motion_vectors_callback(self, msg):
        """Process incoming optical flow data"""
        # Extract motion vectors
        x = np.array(msg.x)
        y = np.array(msg.y)
        
        # Calculate flow values
        # Scale the raw motion values by altitude
        combined_coeff = self.flow_coeff * self.altitude
        
        # Sum the flow vectors and scale them
        # These values will be sent to INAV as raw flow values
        # INAV will apply its own scaling factor (opflow_scale)
        self.last_motion_x = int(np.sum(x) * 1000)  # Scale up for better precision
        self.last_motion_y = int(np.sum(y) * 1000)  # Scale up for better precision
        
        # Set quality based on number of vectors and their magnitude
        if len(x) > 0 and len(y) > 0:
            # Calculate quality based on number of vectors and their magnitude
            magnitude = np.sqrt(np.mean(x**2) + np.mean(y**2))
            # Higher magnitude and more vectors = higher quality
            vector_count_quality = min(255, int(len(x) * 2))
            magnitude_quality = min(255, int(magnitude * 50))
            self.last_quality = min(255, max(100, (vector_count_quality + magnitude_quality) // 2))
        else:
            # No vectors detected
            self.last_quality = 0
        
        # Log flow data occasionally
        now = rospy.Time.now()
        if (now - self.last_send_time).to_sec() >= 1.0:  # Log once per second
            rospy.loginfo(f"Flow: X={self.last_motion_x}, Y={self.last_motion_y}, quality={self.last_quality}, altitude={self.altitude:.3f}m")
            self.last_send_time = now
            
        # Check if altitude data is recent
        duration_from_last_altitude = now - self.altitude_ts
        if duration_from_last_altitude.to_sec() > 10:
            rospy.logwarn(f"No altitude received for {duration_from_last_altitude.to_sec():.1f} seconds.")

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
            
            # Pack data according to MSP protocol
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
                    rospy.loginfo(f"Successfully sent {self.send_count} flow updates. Latest: X={flow_data['motionX']}, Y={flow_data['motionY']}, Q={flow_data['quality']}")
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