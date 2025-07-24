#!/usr/bin/env python3

import rospy
import time
import struct
import numpy as np
import os
import glob
import serial
from serial.serialutil import SerialException
from std_msgs.msg import Float32MultiArray
from unavlib.control.uavcontrol import UAVControl
from unavlib import MSPy

# Format for MSP2_SENSOR_OPTIC_FLOW data
# <Bff: < (little endian), B (unsigned char - quality), f (float - motionX), f (float - motionY)
MSP2_FLOW_FORMAT = '<Bff'

# MSP codes for optic flow data
MSP2_SENSOR_OPTIC_FLOW = 0x1F02  # 7938 decimal - correct code for optical flow

class INAVFlowRelay:
    """
    Relay for optical flow data from C++ node to INAV flight controller.
    Subscribes to optical_flow/inav_data topic and sends data to INAV via MSP protocol.
    Uses uNAVlib for communication with the flight controller.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('inav_flow_relay')
        
        # Flight controller connection parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.update_rate = rospy.get_param('~update_rate', 50)  # Hz
        self.debug = rospy.get_param('~debug', False)  # Debug mode
        self.reconnect_delay = rospy.get_param('~reconnect_delay', 1.0)  # Seconds between reconnection attempts
        
        # Connection state
        self.board = None
        self.connected = False
        self.last_reconnect_attempt = 0
        self.connection_attempts = 0
        
        # Connect to flight controller
        self.connect_to_fc()
        
        # Flow data
        self.last_quality = 255  # Default quality (0-255)
        self.last_motion_x = 0.0
        self.last_motion_y = 0.0
        
        # Subscribe to optical flow data from C++ node
        # This topic is published by the optical_flow.cpp nodelet
        rospy.Subscriber('optical_flow/inav_data', Float32MultiArray, self.flow_callback, queue_size=1)
        
        # Timer for sending data to flight controller
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_flow_data)
        
        # Stats
        self.send_count = 0
        self.last_send_time = rospy.Time.now()
        
        rospy.loginfo("INAV Flow Relay initialized with uNAVlib")
    
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
            
            # Print board capabilities in debug mode
            if self.debug:
                self.print_board_info()
                
            self.connected = True
            self.connection_attempts = 0
            return True
                
        except Exception as e:
            rospy.logerr(f"Failed to connect to flight controller on {port}: {e}")
            self.connected = False
            return False
    
    def print_board_info(self):
        """Print board information for debugging"""
        try:
            # Get board info
            if hasattr(self.board.board, 'SENSOR_DATA'):
                rospy.loginfo(f"SENSOR_DATA available")
            else:
                rospy.loginfo("SENSOR_DATA not available")
                
            # Print available MSP codes
            if hasattr(MSPy, 'MSPCodes'):
                rospy.loginfo("MSPy.MSPCodes is available")
                # Check if optical flow code exists
                if 'MSP2_SENSOR_OPTIC_FLOW' in MSPy.MSPCodes:
                    rospy.loginfo(f"MSP2_SENSOR_OPTIC_FLOW code: {MSPy.MSPCodes['MSP2_SENSOR_OPTIC_FLOW']}")
                else:
                    rospy.loginfo("MSP2_SENSOR_OPTIC_FLOW code not found in MSPy.MSPCodes, using hardcoded value")
                    
            # Get sensor configuration
            self.board.get_sensor_config()
            if hasattr(self.board, 'sensors'):
                rospy.loginfo(f"Optical flow sensor: {self.board.sensors.get('opticalFlowSensor', 'unknown')}")
                rospy.loginfo(f"Available sensors: {self.board.sensors}")
        except Exception as e:
            rospy.logwarn(f"Could not print board info: {e}")
    
    def flow_callback(self, msg):
        """Process incoming optical flow data from C++ node"""
        try:
            # Data format from optical_flow.cpp: [quality, motionX, motionY]
            if len(msg.data) >= 3:
                # Убедимся, что quality находится в диапазоне 0-255
                quality = int(msg.data[0])
                self.last_quality = max(0, min(255, quality))
                self.last_motion_x = float(msg.data[1])
                self.last_motion_y = float(msg.data[2])
                
                # Log flow data occasionally in debug mode
                if self.debug and (rospy.Time.now() - self.last_send_time).to_sec() >= 1.0:
                    rospy.loginfo(f"Received flow: X={self.last_motion_x:.6f}, Y={self.last_motion_y:.6f}, Q={self.last_quality}")
                    self.last_send_time = rospy.Time.now()
        except Exception as e:
            rospy.logerr(f"Error processing flow data: {e}")
            if self.debug:
                import traceback
                rospy.logerr(traceback.format_exc())
    
    def send_flow_data(self, event=None):
        """Send optical flow data to flight controller using MSP protocol"""
        # If not connected or no board, try to reconnect
        if not self.connected or self.board is None:
            self.connect_to_fc()
            return
            
        try:
            # Убедимся, что quality находится в диапазоне 0-255
            quality = max(0, min(255, self.last_quality))
            
            # Отладочный вывод для отслеживания значений
            if self.debug:
                rospy.logdebug(f"Packing data: quality={quality} (type: {type(quality)}), " +
                              f"motion_x={self.last_motion_x} (type: {type(self.last_motion_x)}), " +
                              f"motion_y={self.last_motion_y} (type: {type(self.last_motion_y)})")
            
            # Pack data according to MSP protocol
            # Format: <Bff - quality (byte), motionX (float), motionY (float)
            packed_data = struct.pack(
                MSP2_FLOW_FORMAT, 
                quality,  # Используем проверенное значение
                self.last_motion_x,
                self.last_motion_y
            )
            
            # Send using the direct board object
            result = self.board.board.send_RAW_msg(MSP2_SENSOR_OPTIC_FLOW, data=packed_data)
            
            if result > 0:
                # For MSP2_SENSOR_OPTIC_FLOW, we don't expect a response
                # The flight controller just receives the data
                self.send_count += 1
                
                # Log success message periodically in debug mode
                if self.debug and self.send_count % 40 == 0:  # Log every 40 successful sends (approximately once per second)
                    rospy.loginfo(f"Successfully sent {self.send_count} flow updates. Latest: X={self.last_motion_x:.6f}, Y={self.last_motion_y:.6f}, Q={quality}")
            else:
                # Always log failures
                rospy.logwarn("Failed to send optical flow data")
            
        except SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            self.connected = False
            rospy.loginfo("Port disconnected, will try to reconnect")
            
        except Exception as e:
            rospy.logerr(f"Error sending optical flow data: {e}")
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
                rospy.loginfo(f"Disconnected from flight controller after sending {self.send_count} flow updates")
            except Exception as e:
                rospy.logwarn(f"Error disconnecting from flight controller: {e}")

def main():
    relay = INAVFlowRelay()
    
    # Register shutdown hook
    rospy.on_shutdown(relay.shutdown)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass