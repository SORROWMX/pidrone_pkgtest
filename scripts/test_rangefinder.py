#!/usr/bin/env python

"""
Test script to verify rangefinder data transmission to INAV.
This script subscribes to /pidrone/range and forwards the data to the flight controller.
"""

import rospy
from sensor_msgs.msg import Range
from inavmspapi import MultirotorControl
from inavmspapi.transmitter import SerialTransmitter
from inavmspapi.rangefinder import RangefinderManager
import time
import sys

class RangefinderTest:
    def __init__(self):
        # Connect to the flight controller
        try:
            self.connect_to_board()
        except Exception as e:
            rospy.logerr(f"Failed to connect to flight controller: {e}")
            sys.exit(1)
            
        # Initialize rangefinder manager
        self.rangefinder = RangefinderManager(self.board)
        
        # Subscribe to range topic
        rospy.Subscriber("/pidrone/range", Range, self.range_callback)
        
        # Statistics
        self.packets_sent = 0
        self.last_range = 0
        self.start_time = time.time()
        
    def connect_to_board(self):
        """Connect to the flight controller board"""
        try:
            transmitter = SerialTransmitter('/dev/ttyACM0')
            self.board = MultirotorControl(transmitter)
            if self.board.connect():
                rospy.loginfo("Connected to flight controller on /dev/ttyACM0")
                return
            else:
                raise Exception("Failed to connect to /dev/ttyACM0")
        except Exception as e:
            rospy.logwarn(f"Failed to connect on /dev/ttyACM0: {e}")
            try:
                transmitter = SerialTransmitter('/dev/ttyACM1')
                self.board = MultirotorControl(transmitter)
                if self.board.connect():
                    rospy.loginfo("Connected to flight controller on /dev/ttyACM1")
                    return
                else:
                    raise Exception("Failed to connect to /dev/ttyACM1")
            except Exception as e2:
                raise Exception(f"Could not connect to flight controller: {e2}")
    
    def range_callback(self, msg):
        """Process range data and send to flight controller"""
        range_m = msg.range
        
        if range_m > 0:
            # Send range data to flight controller
            success = self.rangefinder.send_range(range_m)
            
            if success:
                self.packets_sent += 1
                self.last_range = range_m
                
                # Print statistics every 50 packets
                if self.packets_sent % 50 == 0:
                    elapsed = time.time() - self.start_time
                    rate = self.packets_sent / elapsed if elapsed > 0 else 0
                    rospy.loginfo(f"Sent {self.packets_sent} packets ({rate:.1f} Hz), last range: {range_m:.3f}m")
            else:
                rospy.logwarn("Failed to send range data")
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Rangefinder test started. Waiting for range data...")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Test stopped by user")
        finally:
            # Clean up
            if hasattr(self, 'board'):
                self.board.__exit__()
            
            # Print final statistics
            elapsed = time.time() - self.start_time
            rate = self.packets_sent / elapsed if elapsed > 0 else 0
            rospy.loginfo(f"Test complete. Sent {self.packets_sent} packets over {elapsed:.1f} seconds ({rate:.1f} Hz)")

if __name__ == "__main__":
    rospy.init_node('test_rangefinder')
    test = RangefinderTest()
    test.run() 