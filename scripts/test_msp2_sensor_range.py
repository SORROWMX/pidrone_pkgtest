#!/usr/bin/env python

import rospy
import time
import struct
from sensor_msgs.msg import Range
from unavlib.control.uavcontrol import UAVControl
from unavlib import MSPy

# Format for MSP2_SENSOR_RANGEFINDER data
# <Bi: < (little endian), B (unsigned char - quality), i (signed int - distance in mm)
MSP2_RANGE_FORMAT = '<Bi'

# MSP codes for rangefinder data
# Correct code from uNAVlib documentation
MSP2_SENSOR_RANGEFINDER = 0x1F01  # 7937 decimal - correct code for rangefinder
# Note: 0x1F05 (7941) is MSP2_SENSOR_BAROMETER, not rangefinder!

class RangefinderRelay:
    """
    Subscribes to ROS rangefinder topic and forwards data to flight controller
    using the MSP2_SENSOR_RANGEFINDER command
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('rangefinder_relay')
        
        # Flight controller connection parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.update_rate = rospy.get_param('~update_rate', 40)  # Hz - increased to 40Hz for better performance
        
        # Connect to flight controller
        self.board = None
        self.connect_to_fc()
        
        # Last received range data
        self.last_range = None
        self.last_quality = 255  # Default quality (0-255)
        
        # Subscribe to rangefinder topic
        rospy.Subscriber('/pidrone/range', Range, self.range_callback)
        
        # Timer for sending data to flight controller
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_range_data)
        
        rospy.loginfo("Rangefinder relay initialized")
        
        # Print board capabilities
        self.print_board_info()
        
        # Tracking successful sends
        self.send_count = 0
        self.last_send_time = rospy.Time.now()

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
                # Check if rangefinder code exists
                if 'MSP2_SENSOR_RANGEFINDER' in MSPy.MSPCodes:
                    rospy.loginfo(f"MSP2_SENSOR_RANGEFINDER code: {MSPy.MSPCodes['MSP2_SENSOR_RANGEFINDER']}")
                else:
                    rospy.loginfo("MSP2_SENSOR_RANGEFINDER code not found in MSPy.MSPCodes")
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

    def range_callback(self, msg):
        """Process incoming rangefinder data"""
        # If measurement is outside the valid sensor range
        if msg.range < msg.min_range or msg.range > msg.max_range:
            self.last_range = -1  # Negative value means "out of range"
            self.last_quality = 100  # Lower quality for out-of-bounds measurements
        else:
            # Normal measurement within range
            self.last_range = int(msg.range * 1000)  # Convert meters to millimeters
            self.last_quality = 255  # Maximum quality
        
        # Only log range data occasionally to reduce log spam
        now = rospy.Time.now()
        if (now - self.last_send_time).to_sec() >= 1.0:  # Log once per second
            if self.last_range < 0:
                rospy.loginfo(f"Current range: OUT OF RANGE, quality: {self.last_quality}")
            else:
                rospy.loginfo(f"Current range: {msg.range:.3f}m ({self.last_range}mm), quality: {self.last_quality}")
            self.last_send_time = now

    def send_range_data(self, event=None):
        """Send rangefinder data to flight controller"""
        if self.last_range is None or self.board is None:
            return
            
        try:
            # Prepare range data message
            range_data = {
                'quality': self.last_quality,
                'distanceMm': self.last_range  # Either valid distance in mm or -1 for out of range
            }
            
            # Pack data according to MSP protocol
            packed_data = struct.pack(MSP2_RANGE_FORMAT, 
                                      range_data['quality'], 
                                      range_data['distanceMm'])
            
            # Use direct board access for sending MSP commands
            msp_code = MSP2_SENSOR_RANGEFINDER
            
            # Send using the direct board object
            result = self.board.board.send_RAW_msg(msp_code, data=packed_data)
            
            if result > 0:
                # For MSP2_SENSOR_RANGEFINDER, we don't expect a response
                # The flight controller just receives the data
                self.send_count += 1
                
                # Log success message periodically to avoid flooding logs
                if self.send_count % 40 == 0:  # Log every 40 successful sends (approximately once per second)
                    if self.last_range < 0:
                        rospy.loginfo(f"Successfully sent {self.send_count} rangefinder updates. Latest: OUT OF RANGE")
                    else:
                        rospy.loginfo(f"Successfully sent {self.send_count} rangefinder updates. Latest: {range_data['distanceMm']}mm ({range_data['distanceMm']/1000.0:.2f}m)")
            else:
                rospy.logwarn("Failed to send rangefinder data")
            
        except Exception as e:
            rospy.logerr(f"Error sending rangefinder data: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            
    def shutdown(self):
        """Clean up when node is shutting down"""
        if self.board:
            try:
                self.board.disconnect()
                rospy.loginfo(f"Disconnected from flight controller after sending {self.send_count} rangefinder updates")
            except:
                pass

def main():
    relay = RangefinderRelay()
    
    # Register shutdown hook
    rospy.on_shutdown(relay.shutdown)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 