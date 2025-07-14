#!/usr/bin/env python

"""
Test script for getting hover_throttle through MSP protocol
and integrating this value with PID controller.
"""

import sys
import time
import rospy
from h2rMultiWii import MultiWii
from pid_class_new import PID, PIDaxis, HOVER_THROTTLE, DEADBAND

def main():
    # Initialize ROS node
    rospy.init_node('test_hover_throttle', anonymous=True)
    
    # Serial port can be passed as a command line argument
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        # Default to /dev/ttyACM0
        serial_port = '/dev/ttyACM0'
    
    print("Connecting to flight controller via port %s..." % serial_port)
    
    try:
        # Initialize connection to flight controller
        board = MultiWii(serial_port)
        
        # Get firmware version information
        print("Requesting controller identification...")
        ident = board.getData(MultiWii.IDENT)
        print("Controller information: %s" % ident)
        
        # Get current hover_throttle value from INAV via MSP
        print("\nStarting hover_throttle test...")
        hover_throttle_inav = board.test_get_hover_throttle()
        
        # Get current value from PID controller settings
        print("\nCurrent values in PID controller:")
        print("HOVER_THROTTLE = %d" % HOVER_THROTTLE)
        print("DEADBAND = %d" % DEADBAND)
        
        # Check if values match
        if hover_throttle_inav is not None:
            if hover_throttle_inav == HOVER_THROTTLE:
                print("\nValues match: PID controller already uses the correct hover_throttle value")
            else:
                print("\nValues do not match:")
                print("  - INAV hover_throttle: %d" % hover_throttle_inav)
                print("  - PID hover_throttle: %d" % HOVER_THROTTLE)
                print("\nRecommended to update HOVER_THROTTLE value in pid_class_new.py")
        
        # Get altitude sensor data
        print("\nRequesting barometer data...")
        altitude_data = board.getData(MultiWii.ALTITUDE)
        print("Altitude data: %s" % altitude_data)
        
        # Test PID controller with current settings
        print("\nCreating test PID controller with current settings...")
        test_pid = PID()
        
        # Check hover_throttle value in PID controller
        print("Hover_throttle value in created PID: %d" % test_pid.throttle.hover_throttle)
        print("Deadband value in created PID: %d" % test_pid.throttle.deadband)
        
        # Create test case for PID controller
        if altitude_data and 'estalt' in altitude_data:
            current_height = altitude_data['estalt'] / 100.0  # convert from cm to m
            
            # Simulate PID step for current height
            from three_dim_vec import Error
            error = Error(0, 0, 0.65 - current_height)
            
            print("\nTesting PID controller with current height: %fm" % current_height)
            print("Target height: 0.65m, error: %fm" % error.z)
            
            # Execute one step of PID controller
            cmd = test_pid.step(error)
            print("PID.step() result: %s" % cmd)
            print("Throttle command: %d" % cmd[2])
            
            # For testing what happens if drone is on the ground
            ground_error = Error(0, 0, 0.65 - 0.02)  # drone on ground, height 2cm
            print("\nTesting with simulated drone on ground (height 2cm):")
            ground_cmd = test_pid.step(ground_error)
            print("PID.step() result for drone on ground: %s" % ground_cmd)
            print("Throttle command: %d" % ground_cmd[2])
            
    except Exception as e:
        import traceback
        print("Error during test execution: %s" % e)
        traceback.print_exc()
    finally:
        # Close connection to flight controller
        if 'board' in locals():
            board.close()
            print("Connection to flight controller closed")

if __name__ == "__main__":
    main() 