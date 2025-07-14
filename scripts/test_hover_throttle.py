#!/usr/bin/env python

"""
Test script for getting hover_throttle through MSP_NAV_POSHOLD command
from INAV flight controller.
"""

import sys
import time
import rospy
from h2rMultiWii import MultiWii
from pid_class_new import PID, PIDaxis, HOVER_THROTTLE, DEADBAND
from serial import SerialException

def main():
    # Initialize ROS node
    rospy.init_node('test_hover_throttle', anonymous=True)
    
    # Serial port can be passed as a command line argument
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        # Default to /dev/ttyACM0
        serial_port = '/dev/ttyACM0'
    
    print("\n===== HOVER THROTTLE TEST =====")
    print("Connecting to flight controller via port %s..." % serial_port)
    
    try:
        # Initialize connection to flight controller
        board = MultiWii(serial_port)
        print("Connected to flight controller")
        
        # Get current hover_throttle value from INAV via MSP_NAV_POSHOLD
        print("\nRequesting hover_throttle value from flight controller...")
        hover_throttle_inav = board.get_hover_throttle()
        
        if hover_throttle_inav is not None:
            print("\nSuccessfully received hover_throttle value")
            print("INAV hover_throttle: %d μs" % hover_throttle_inav)
            
            # Get current value from PID controller settings
            print("\nCurrent values in PID controller:")
            print("HOVER_THROTTLE = %d μs" % HOVER_THROTTLE)
            print("DEADBAND = %d μs" % DEADBAND)
            
            # Check if values match
            if hover_throttle_inav == HOVER_THROTTLE:
                print("\nHover throttle values match ✓")
                print("PID controller already uses the correct hover_throttle value")
            else:
                print("\nHover throttle values DO NOT match ✗")
                print("  - INAV hover_throttle: %d μs" % hover_throttle_inav)
                print("  - PID hover_throttle: %d μs" % HOVER_THROTTLE)
                print("\nRecommendation: Update HOVER_THROTTLE value in pid_class_new.py to match INAV's value")
        else:
            print("\nFailed to get hover_throttle value from flight controller")
            print("Make sure the flight controller is correctly connected and running INAV firmware")
            
        # Try to get additional information about the NAV_POSHOLD settings
        print("\nRequesting additional NAV_POSHOLD data...")
        board.getData(MultiWii.MSP_NAV_POSHOLD)
        
        if hasattr(board, 'navPoshold'):
            print("\nComplete NAV_POSHOLD data:")
            for key, value in board.navPoshold.items():
                if key not in ['timestamp', 'elapsed', 'cmd']:
                    print("  %s: %s" % (key, value))
        
    except SerialException as e:
        print("\nError connecting to flight controller: %s" % e)
        print("Make sure the flight controller is connected to %s" % serial_port)
    except Exception as e:
        import traceback
        print("\nError during execution: %s" % e)
        traceback.print_exc()
    finally:
        # Close connection to flight controller
        if 'board' in locals():
            board.close()
            print("\nConnection to flight controller closed")
        print("\n===== TEST COMPLETED =====")

if __name__ == "__main__":
    main() 