#!/usr/bin/env python3

import rospy
import time
import struct
import numpy as np
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
        self.update_rate = rospy.get_param('~update_rate', 40)  # Hz
        self.debug = rospy.get_param('~debug', False)  # Debug mode
        
        # Connect to flight controller
        self.board = None
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
    
    def connect_to_fc(self):
        """Connect to the flight controller board with improved error handling"""
        try:
            rospy.loginfo(f"Connecting to flight controller on {self.serial_port}")
            self.board = UAVControl(self.serial_port, self.baudrate, receiver="serial")
            self.board.connect()
            rospy.loginfo("Connected to flight controller")
            
            # Print board capabilities in debug mode
            if self.debug:
                self.print_board_info()
                
        except Exception as e:
            rospy.logerr(f"Failed to connect to flight controller on primary port: {e}")
            try:
                # Try alternative port
                alt_port = '/dev/ttyACM1'
                rospy.loginfo(f"Trying alternative port {alt_port}")
                self.board = UAVControl(alt_port, self.baudrate, receiver="serial")
                self.board.connect()
                rospy.loginfo("Connected to flight controller on alternative port")
                
                # Print board capabilities in debug mode
                if self.debug:
                    self.print_board_info()
                    
            except Exception as e:
                rospy.logerr(f"Failed to connect to flight controller on alternative port: {e}")
                rospy.signal_shutdown("Could not connect to flight controller")
    
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
        if self.board is None:
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