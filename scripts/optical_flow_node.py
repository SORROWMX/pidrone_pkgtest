#!/usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from raspicam_node.msg import MotionVectors
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty, Float32



class OpticalFlowNode(object):
    """
    Subscribe to the optical flow vectors and publish linear velocity as a Twist message.
    """
    def __init__(self, node_name):

        rospy.init_node(node_name)
        # flow variables
        camera_wh = (320, 240)        
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)
        
        # Altitude variables
        self.range_altitude = 0.03 # initialize rangefinder to a bit off the ground
        self.baro_altitude = None  # barometer altitude (will be set when data is received)
        self.altitude_ts = rospy.Time.now()
        
        # Sensor fusion weights
        self.range_weight = 0.7  # Range finder more accurate at close range
        self.baro_weight = 0.3   # Barometer less accurate but works at all altitudes
        
        # Sensor health monitoring
        self.last_range_time = rospy.Time.now()
        self.last_baro_time = None
        self.range_timeout = rospy.Duration.from_sec(1.0)
        self.baro_timeout = rospy.Duration.from_sec(1.0)
        
        self.setup()

    def setup(self):
        # publisher
        self.twistpub = rospy.Publisher('/pidrone/picamera/twist', TwistStamped, queue_size=1)

        # subscribers
        self._sub_mv = rospy.Subscriber('/raspicam_node/motion_vectors', MotionVectors, self.motion_cb, queue_size=1)
        self._sub_alt = rospy.Subscriber('/pidrone/range', Range, self.range_cb, queue_size=1)
        # Subscribe to barometer altitude data
        self._sub_baro = rospy.Subscriber('/pidrone/altitude', Float32, self.baro_cb, queue_size=1)

    def motion_cb(self, msg):
        ''' Average the motion vectors and publish the
        twist message. 
        '''
        # signed 1-byte values
        x = msg.x
        y = msg.y

        # Get the best altitude estimate for optical flow calculation
        altitude = self.get_best_altitude()
        
        # Calculate the planar and yaw motions
        # Calculate coefficient once
        combined_coeff = self.flow_coeff * altitude
        x_motion = np.sum(x) * combined_coeff
        y_motion = np.sum(y) * combined_coeff
        
        # Create and publish twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = x_motion
        twist_msg.twist.linear.y = -y_motion
        
        # Debug output (uncomment for debugging)
        #print(f"Alt: {altitude:.2f}m, Flow: {x_motion:.2f}, {-y_motion:.2f}")
        
        # Update and publish the twist message
        self.twistpub.publish(twist_msg)
        
        # Check for missing altitude data
        current_time = rospy.Time.now()
        if current_time - self.altitude_ts > rospy.Duration.from_sec(5):
            rospy.logwarn("No altitude data received for {:10.4f} seconds.".format((current_time - self.altitude_ts).to_sec()))

    def range_cb(self, msg):
        """
        Rangefinder altitude callback
        """
        self.range_altitude = msg.range
        self.last_range_time = rospy.Time.now()
        self.altitude_ts = self.last_range_time
    
    def baro_cb(self, msg):
        """
        Barometer altitude callback
        """
        self.baro_altitude = msg.data
        self.last_baro_time = rospy.Time.now()
        # Only update timestamp if we don't have recent range data
        if rospy.Time.now() - self.last_range_time > self.range_timeout:
            self.altitude_ts = self.last_baro_time
    
    def get_best_altitude(self):
        """
        Determine the best altitude estimate based on available sensors.
        Combines rangefinder and barometer data when both are available.
        Returns rangefinder data only when flying close to ground.
        Returns barometer data when rangefinder is out of range.
        """
        current_time = rospy.Time.now()
        range_valid = (current_time - self.last_range_time < self.range_timeout)
        baro_valid = (self.baro_altitude is not None and 
                     (self.last_baro_time is not None) and 
                     (current_time - self.last_baro_time < self.baro_timeout))
                     
        # Both sensors valid - use weighted average
        if range_valid and baro_valid:
            # Use more weight for range when close to ground (< 1.0m)
            if self.range_altitude < 1.0:
                # When very close to ground, trust range finder more
                range_w = 0.8
                baro_w = 0.2
            else:
                # At higher altitudes, standard weights
                range_w = self.range_weight
                baro_w = self.baro_weight
            
            return range_w * self.range_altitude + baro_w * self.baro_altitude
        
        # Only range is valid
        elif range_valid:
            return self.range_altitude
        
        # Only barometer is valid
        elif baro_valid:
            return self.baro_altitude
        
        # No valid sensors - use last known altitude or default
        return max(0.03, self.range_altitude)  # Minimum 3cm to prevent division by zero issues
    
def main():
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()

if __name__ == '__main__':
    main()
