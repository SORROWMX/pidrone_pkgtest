#!/usr/bin/env python

import rospy
import sys
import time
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

class AltitudeControlClient:
    """
    Client for testing the altitude controller services.
    Provides a simple command line interface to test different altitude control modes.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('altitude_control_client')
        
        # Current and target altitude
        self.current_altitude = 0.0
        self.target_altitude = 0.0
        
        # Subscribe to altitude data
        rospy.Subscriber('/pidrone/range', Range, self.altitude_callback, queue_size=1)
        rospy.Subscriber('/altitude_controller/target', Float32, self.target_callback, queue_size=1)
        
        # Wait for services to become available
        rospy.loginfo("Waiting for altitude control services...")
        try:
            rospy.wait_for_service('/altitude_controller/hold_current', timeout=5.0)
            rospy.wait_for_service('/altitude_controller/constant_climb', timeout=5.0)
            rospy.wait_for_service('/altitude_controller/constant_descent', timeout=5.0)
            rospy.wait_for_service('/altitude_controller/set_target', timeout=5.0)
            
            # Create service proxies
            self.hold_current_srv = rospy.ServiceProxy('/altitude_controller/hold_current', Empty)
            self.constant_climb_srv = rospy.ServiceProxy('/altitude_controller/constant_climb', Empty)
            self.constant_descent_srv = rospy.ServiceProxy('/altitude_controller/constant_descent', Empty)
            self.set_target_srv = rospy.ServiceProxy('/altitude_controller/set_target', Empty)
            
            rospy.loginfo("All services available")
        except rospy.ROSException as e:
            rospy.logerr(f"Service wait timed out: {e}")
            sys.exit(1)
            
        # Print instructions
        self.print_instructions()
        
    def altitude_callback(self, msg):
        """Process incoming altitude data"""
        self.current_altitude = msg.range
        
    def target_callback(self, msg):
        """Process incoming target altitude data"""
        self.target_altitude = msg.data
        
    def print_instructions(self):
        """Print instructions for using the client"""
        print("\n=== Altitude Control Test Client ===")
        print("Commands:")
        print("  h - Hold current altitude")
        print("  c - Start constant climb")
        print("  d - Start constant descent")
        print("  t <altitude> - Set target altitude (in meters)")
        print("  s - Show current altitude status")
        print("  q - Quit")
        print("================================")
        
    def print_status(self):
        """Print current altitude status"""
        print(f"\nCurrent altitude: {self.current_altitude:.2f}m")
        print(f"Target altitude: {self.target_altitude:.2f}m")
        
    def set_parameter_target(self, altitude):
        """Set the target altitude parameter"""
        try:
            altitude_float = float(altitude)
            if altitude_float < 0:
                print("Warning: Target altitude is negative. This might not be supported.")
            rospy.set_param('/altitude_controller/target_altitude', altitude_float)
            print(f"Target altitude parameter set to {altitude_float}m")
            return True
        except ValueError:
            print(f"Error: Invalid altitude value '{altitude}'. Please enter a number.")
            return False
        
    def run(self):
        """Run the client command loop"""
        while not rospy.is_shutdown():
            try:
                # Get user input
                cmd = input("\nEnter command (h/c/d/t/s/q): ").strip()
                
                if not cmd:
                    continue
                    
                if cmd.lower() == 'q':
                    print("Exiting...")
                    break
                    
                elif cmd.lower() == 'h':
                    print("Sending hold current altitude command...")
                    self.hold_current_srv()
                    
                elif cmd.lower() == 'c':
                    print("Sending constant climb command...")
                    self.constant_climb_srv()
                    
                elif cmd.lower() == 'd':
                    print("Sending constant descent command...")
                    self.constant_descent_srv()
                    
                elif cmd.lower() == 's':
                    self.print_status()
                    
                elif cmd.lower().startswith('t '):
                    # Extract altitude value
                    parts = cmd.split(' ', 1)
                    if len(parts) < 2 or not parts[1]:
                        print("Error: Missing altitude value. Format: t <altitude>")
                        continue
                        
                    # Set the parameter and send the command
                    if self.set_parameter_target(parts[1]):
                        print("Sending set target altitude command...")
                        self.set_target_srv()
                        
                else:
                    print(f"Unknown command: {cmd}")
                    self.print_instructions()
                    
            except Exception as e:
                print(f"Error: {e}")
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
                
        print("Client terminated")

def main():
    client = AltitudeControlClient()
    client.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 