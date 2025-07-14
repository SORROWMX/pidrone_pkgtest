#!/usr/bin/env python

"""
Test script for getting hover_throttle through MSP_NAV_POSHOLD command
from INAV flight controller.
"""

import sys
import time
import rospy
from h2rMultiWii import MultiWii
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
    
    try:
        # Initialize connection to flight controller
        board = MultiWii(serial_port)
        
        # Get current hover_throttle value from INAV via MSP_NAV_POSHOLD
        hover_throttle_inav = board.get_hover_throttle()
        
        # Try to get additional information about the NAV_POSHOLD settings
        board.getData(MultiWii.MSP_NAV_POSHOLD)
        
    except SerialException as e:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        # Close connection to flight controller
        if 'board' in locals():
            board.close()

if __name__ == "__main__":
    main() 