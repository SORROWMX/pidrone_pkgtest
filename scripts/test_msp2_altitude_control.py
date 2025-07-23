#!/usr/bin/env python3

import rospy
import time
import struct
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from unavlib.control.uavcontrol import UAVControl
from unavlib import MSPy

# Format for MSP2_INAV_SET_ALTITUDE data
# <fi: < (little endian), f (float - targetAltitude), B (uint8_t - mode)
MSP2_ALTITUDE_FORMAT = '<fB'
# Format with climb rate
MSP2_ALTITUDE_FORMAT_WITH_RATE = '<ffB'  # targetAltitude, climbRate, mode

# MSP codes for altitude control
MSP2_INAV_SET_ALTITUDE = 0x2220  # 8736 decimal

# Altitude control modes
ROC_TO_ALT_CURRENT = 0   # Set current altitude as target (targetAltitude ignored)
ROC_TO_ALT_CONSTANT = 1  # Constant rate of climb/descent (targetAltitude ignored)
ROC_TO_ALT_TARGET = 2    # Move to target altitude (targetAltitude used)

class AltitudeController:
    """
    Controls drone altitude using MSP2_INAV_SET_ALTITUDE command.
    Provides ROS services for different altitude control modes.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('altitude_controller')
        
        # Flight controller connection parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        
        # Connect to flight controller
        self.board = None
        self.connect_to_fc()
        
        # Current altitude
        self.current_altitude = 0.0
        
        # Subscribe to altitude data
        rospy.Subscriber('/pidrone/range', Range, self.altitude_callback, queue_size=1)
        
        # Publisher for target altitude
        self.target_pub = rospy.Publisher('/altitude_controller/target', Float32, queue_size=1)
        
        # Services for altitude control
        rospy.Service('/altitude_controller/hold_current', Empty, self.hold_current_altitude)
        rospy.Service('/altitude_controller/constant_climb', Empty, self.constant_climb)
        rospy.Service('/altitude_controller/constant_descent', Empty, self.constant_descent)
        rospy.Service('/altitude_controller/set_target', Empty, self.set_target_altitude)
        
        # Target altitude parameter
        self.target_altitude = rospy.get_param('~target_altitude', 1.0)  # Default target: 1 meter
        
        rospy.loginfo("Altitude Controller initialized")
        rospy.loginfo("Default target altitude: {} meters".format(self.target_altitude))
        
        # Print board capabilities
        self.print_board_info()

    def print_board_info(self):
        """Print board information for debugging"""
        try:
            # Get board info
            if hasattr(self.board.board, 'SENSOR_DATA'):
                rospy.loginfo("SENSOR_DATA available: {}".format(self.board.board.SENSOR_DATA))
            else:
                rospy.loginfo("SENSOR_DATA not available")
                
            # Print available MSP codes
            if hasattr(MSPy, 'MSPCodes'):
                rospy.loginfo("MSPy.MSPCodes is available")
                # Check if altitude control code exists
                if 'MSP2_INAV_SET_ALTITUDE' in MSPy.MSPCodes:
                    rospy.loginfo("MSP2_INAV_SET_ALTITUDE code: {}".format(MSPy.MSPCodes['MSP2_INAV_SET_ALTITUDE']))
                else:
                    rospy.loginfo("MSP2_INAV_SET_ALTITUDE code not found in MSPy.MSPCodes")
            else:
                rospy.loginfo("MSPy.MSPCodes not available")
        except Exception as e:
            rospy.logwarn("Could not print board info: {}".format(e))

    def connect_to_fc(self):
        """Connect to the flight controller board"""
        try:
            rospy.loginfo("Connecting to flight controller on {}".format(self.serial_port))
            self.board = UAVControl(self.serial_port, self.baudrate, receiver="serial")
            self.board.connect()
            rospy.loginfo("Connected to flight controller")
        except Exception as e:
            rospy.logerr("Failed to connect to flight controller: {}".format(e))
            try:
                # Try alternative port
                alt_port = '/dev/ttyACM1'
                rospy.loginfo("Trying alternative port {}".format(alt_port))
                self.board = UAVControl(alt_port, self.baudrate, receiver="serial")
                self.board.connect()
                rospy.loginfo("Connected to flight controller on alternative port")
            except Exception as e:
                rospy.logerr("Failed to connect to flight controller on alternative port: {}".format(e))
                rospy.signal_shutdown("Could not connect to flight controller")

    def altitude_callback(self, msg):
        """Process incoming altitude data"""
        self.current_altitude = msg.range
        
    def send_altitude_command(self, target_altitude, mode, climb_rate=None):
        """Send altitude control command to flight controller"""
        if self.board is None:
            rospy.logerr("Cannot send altitude command: not connected to flight controller")
            return False
            
        try:
            # Prepare altitude data message
            altitude_data = {
                'targetAltitude': float(target_altitude),
                'mode': mode
            }
            
            # Pack data according to MSP protocol
            if climb_rate is not None:
                # Use format with 9 bytes, including climb rate
                packed_data = struct.pack(MSP2_ALTITUDE_FORMAT_WITH_RATE, 
                                      float(target_altitude), 
                                      float(climb_rate),
                                      mode)
            else:
                # Use format with 5 bytes
                packed_data = struct.pack(MSP2_ALTITUDE_FORMAT, 
                                      float(target_altitude), 
                                      mode)
            
            # Use direct board access for sending MSP commands
            msp_code = MSP2_INAV_SET_ALTITUDE
            
            # Send using the direct board object
            result = self.board.board.send_RAW_msg(msp_code, data=packed_data)
            
            if result > 0:
                if climb_rate is not None:
                    rospy.loginfo("Altitude command sent: mode={}, target={}m, climb rate={}m/s".format(mode, target_altitude, climb_rate))
                else:
                    rospy.loginfo("Altitude command sent: mode={}, target={}m".format(mode, target_altitude))
                
                # Publish target altitude for visualization/monitoring
                target_msg = Float32()
                target_msg.data = float(target_altitude)
                self.target_pub.publish(target_msg)
                
                return True
            else:
                rospy.logwarn("Failed to send altitude command")
                return False
            
        except Exception as e:
            rospy.logerr("Error sending altitude command: {}".format(e))
            import traceback
            rospy.logerr(traceback.format_exc())
            return False
    
    # Service callbacks for different altitude control modes
    
    def hold_current_altitude(self, req):
        """Service callback to hold current altitude"""
        rospy.loginfo("Holding current altitude: {:.2f}m".format(self.current_altitude))
        self.send_altitude_command(0.0, ROC_TO_ALT_CURRENT)
        return EmptyResponse()
    
    def constant_climb(self, req):
        """Service callback for constant climb"""
        climb_rate = rospy.get_param('~climb_rate', 0.5)  # m/s, positive value
        rospy.loginfo("Starting constant climb at {} m/s".format(climb_rate))
        self.send_altitude_command(0.0, ROC_TO_ALT_CONSTANT, climb_rate)
        return EmptyResponse()
    
    def constant_descent(self, req):
        """Service callback for constant descent"""
        descent_rate = -rospy.get_param('~descent_rate', 0.5)  # m/s, negative value
        rospy.loginfo("Starting constant descent at {} m/s".format(-descent_rate))
        self.send_altitude_command(0.0, ROC_TO_ALT_CONSTANT, descent_rate)
        return EmptyResponse()
    
    def set_target_altitude(self, req):
        """Service callback to move to target altitude"""
        # Get the target altitude from parameter
        target = rospy.get_param('~target_altitude', 1.0)
        rospy.loginfo("Moving to target altitude: {}m".format(target))
        self.send_altitude_command(target, ROC_TO_ALT_TARGET)
        return EmptyResponse()
    
    def shutdown(self):
        """Clean up when node is shutting down"""
        if self.board:
            try:
                self.board.disconnect()
                rospy.loginfo("Disconnected from flight controller")
            except:
                pass

def main():
    controller = AltitudeController()
    
    # Register shutdown hook
    rospy.on_shutdown(controller.shutdown)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 