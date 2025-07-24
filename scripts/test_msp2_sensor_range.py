#!/usr/bin/env python

import rospy
import time
import struct
import os
import glob
import serial
from serial.serialutil import SerialException
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
        self.debug = rospy.get_param('~debug', False)  # Debug mode disabled by default
        self.reconnect_delay = rospy.get_param('~reconnect_delay', 1.0)  # Seconds between reconnection attempts
        
        # Connection state
        self.board = None
        self.connected = False
        self.last_reconnect_attempt = 0
        self.connection_attempts = 0
        
        # Connect to flight controller
        self.connect_to_fc()
        
        # Last received range data
        self.last_range = None
        self.last_quality = 255  # Default quality (0-255)
        
        # Subscribe to rangefinder topic
        rospy.Subscriber('/pidrone/range', Range, self.range_callback)
        
        # Timer for sending data to flight controller
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_range_data)
        
        rospy.loginfo("Rangefinder relay initialized")
        
        # Print board capabilities only in debug mode
        if self.debug and self.connected:
            self.print_board_info()
        
        # Tracking successful sends
        self.send_count = 0
        self.last_send_time = rospy.Time.now()

    def find_available_ports(self):
        """Find all available ttyACM ports"""
        ports = []
        for port in glob.glob('/dev/ttyACM*'):
            ports.append(port)
        return ports
    
    def is_port_available(self, port):
        """Check if a port is available"""
        try:
            s = serial.Serial(port)
            s.close()
            return True
        except:
            return False

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
        """Connect to the flight controller board with improved error handling"""
        # Don't try to reconnect too frequently
        current_time = time.time()
        if current_time - self.last_reconnect_attempt < self.reconnect_delay:
            return False
        
        self.last_reconnect_attempt = current_time
        self.connection_attempts += 1
        
        # First try the specified port
        if self.try_connect(self.serial_port):
            return True
            
        # If that fails, try to find any available ttyACM port
        ports = self.find_available_ports()
        rospy.loginfo(f"Found available ports: {ports}")
        
        for port in ports:
            if port != self.serial_port and self.try_connect(port):
                # Update the default port if we successfully connect to a different one
                self.serial_port = port
                return True
        
        # Log if all connection attempts failed
        rospy.logerr("Failed to connect to any available port")
        return False
    
    def try_connect(self, port):
        """Try to connect to a specific port"""
        try:
            # If we already have a board object, try to disconnect first
            if self.board is not None:
                try:
                    self.board.disconnect()
                except:
                    pass
                    
            rospy.loginfo(f"Connecting to flight controller on {port}")
            self.board = UAVControl(port, self.baudrate, receiver="serial")
            self.board.connect()
            rospy.loginfo(f"Successfully connected to flight controller on {port}")
            
            self.connected = True
            self.connection_attempts = 0
            return True
                
        except Exception as e:
            rospy.logerr(f"Failed to connect to flight controller on {port}: {e}")
            self.connected = False
            return False

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
        
        # Only log range data occasionally and only in debug mode
        if self.debug:
            now = rospy.Time.now()
            if (now - self.last_send_time).to_sec() >= 1.0:  # Log once per second
                if self.last_range < 0:
                    rospy.loginfo(f"Current range: OUT OF RANGE, quality: {self.last_quality}")
                else:
                    rospy.loginfo(f"Current range: {msg.range:.3f}m ({self.last_range}mm), quality: {self.last_quality}")
                self.last_send_time = now

    def send_range_data(self, event=None):
        """Send rangefinder data to flight controller"""
        # If not connected or no board, try to reconnect
        if not self.connected or self.board is None:
            self.connect_to_fc()
            return
            
        if self.last_range is None:
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
                
                # Log success message periodically and only in debug mode
                if self.debug and self.send_count % 40 == 0:  # Log every 40 successful sends (approximately once per second)
                    if self.last_range < 0:
                        rospy.loginfo(f"Successfully sent {self.send_count} rangefinder updates. Latest: OUT OF RANGE")
                    else:
                        rospy.loginfo(f"Successfully sent {self.send_count} rangefinder updates. Latest: {range_data['distanceMm']}mm ({range_data['distanceMm']/1000.0:.2f}m)")
            else:
                # Always log failures
                rospy.logwarn("Failed to send rangefinder data")
            
        except SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            self.connected = False
            rospy.loginfo("Port disconnected, will try to reconnect")
            
        except Exception as e:
            rospy.logerr(f"Error sending rangefinder data: {e}")
            if "Attempting to use a port that is not open" in str(e):
                self.connected = False
                rospy.loginfo("Port not open, will try to reconnect")
            elif self.debug:
                import traceback
                rospy.logerr(traceback.format_exc())
            
    def shutdown(self):
        """Clean up when node is shutting down"""
        if self.board:
            try:
                self.board.disconnect()
                if self.debug:
                    rospy.loginfo(f"Disconnected from flight controller after sending {self.send_count} rangefinder updates")
                else:
                    rospy.loginfo("Disconnected from flight controller")
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