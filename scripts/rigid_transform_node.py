#!/usr/bin/env python3


import tf
import time
import cv2
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool, Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from pidrone_pkg.msg import State
from sensor_msgs.msg import Range
from cv_bridge import CvBridge


class RigidTransformNode(object):
    """
    A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform

    Publisher:
    ~pose

    Subscribers:
    ~reset_transform
    ~position_control
    """
    def __init__(self, node_name):
        # initialize the DTROS parent class
        rospy.init_node(node_name)

        camera_wh = (320, 240)

        self.bridge = CvBridge()

        # initialize the Pose data
        self.pose_msg = PoseStamped()
        
        # Altitude data
        self.range_altitude = 0.03  # Initialize range finder altitude
        self.baro_altitude = None   # Barometer altitude
        self.x_position_from_state = 0.0
        self.y_position_from_state = 0.0

        # Sensor health monitoring
        self.last_range_time = rospy.Time.now()
        self.last_baro_time = None
        self.range_timeout = rospy.Duration.from_sec(1.0)
        self.baro_timeout = rospy.Duration.from_sec(1.0)
        
        # Sensor fusion weights
        self.range_weight = 0.7  # Range finder more accurate at close range
        self.baro_weight = 0.3   # Barometer less accurate but works at all altitudes

        # position hold is initialized as False
        self.position_control = False
        self.first_image = None
        self.previous_image = None

        # used as a safety check for position control
        self.consecutive_lost_counter = 0
        self.lost = False

        # first image vars
        self.first = True
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # ROS Setup
        ###########
        # Publisher
        self._posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)
        self._lostpub = rospy.Publisher('/pidrone/picamera/lost', Bool, queue_size=1)
        
        # Subscribers
        self._rtsub = rospy.Subscriber("/pidrone/reset_transform", Empty, self.reset_callback, queue_size=1)
        self._pcsub = rospy.Subscriber("/pidrone/position_control", Bool, self.position_control_callback, queue_size=1)
        self._stsub = rospy.Subscriber("/pidrone/state", State, self.state_callback, queue_size=1)
        self._isub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback, queue_size=1)
        self._sub_range = rospy.Subscriber('/pidrone/range', Range, self.range_callback, queue_size=1)
        self._sub_baro = rospy.Subscriber('/pidrone/altitude', Float32, self.baro_callback, queue_size=1)


    def range_callback(self, msg):
        """
        Update rangefinder altitude
        """
        self.range_altitude = msg.range
        self.last_range_time = rospy.Time.now()
        
    def baro_callback(self, msg):
        """
        Update barometer altitude
        """
        self.baro_altitude = msg.data
        self.last_baro_time = rospy.Time.now()
        
    def get_best_altitude(self):
        """
        Determine the best altitude estimate based on available sensors
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
        return max(0.03, self.range_altitude)

    def image_callback(self, msg):
        ''' A method that is called everytime an image is taken '''

        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="mono8")
        dimage = image.copy()

        # Get the current altitude
        altitude = self.get_best_altitude()
        
        # Run the following only if position control is enabled to prevent
        # wasting computation resources on unused position data
        if self.position_control:
            # if there is no first image stored, tell the user to capture an image
            if self.first:
                self.first = False
                print("Capturing a new first image")
                self.first_image = image
                self.first_points = cv2.goodFeaturesToTrack(self.first_image, maxCorners=10, qualityLevel=0.01, minDistance=8)
                self.previous_image = image
                self.last_first_time = rospy.get_time()
            # if a first image has been stored
            else:
                # try to estimate the transformations from the first image
                nextPts, status, err = cv2.calcOpticalFlowPyrLK(self.first_image, image, self.first_points, None)

                transform_first, inliers = cv2.estimateAffinePartial2D(self.first_points, nextPts, False)

                # if the first image was visible (the transformation was succesful) :
                if transform_first is not None:
                    self.lost = False
                    # calculate the x,y, and yaw translations from the transformation
                    translation_first, yaw_first = self.translation_and_yaw(transform_first)
                    # use an EMA filter to smooth the position and yaw values
                    self.pose_msg.pose.position.x = translation_first[0] * altitude
                    self.pose_msg.pose.position.y = translation_first[1] * altitude
                    # With just a yaw, the x and y components of the
                    # quaternion are 0
                    _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw_first)
                    self.pose_msg.pose.orientation.z = z
                    self.pose_msg.pose.orientation.w = w
                    # update first image data
                    self.first_image_counter += 1
                    self.max_first_counter = max(self.max_first_counter, self.first_image_counter)
                    self.last_first_time = rospy.get_time()
                    print(("count:", self.first_image_counter))
                # else the first image was not visible (the transformation was not succesful) :
                else:
                    # try to estimate the transformation from the previous image
                    transform_previous, inliers = cv2.estimateAffinePartial2D(self.previous_image, image, False)

                    # if the previous image was visible (the transformation was succesful)
                    # calculate the position by integrating
                    if transform_previous is not None:
                        self.lost = False
                        if self.last_first_time is None:
                            self.last_first_time = rospy.get_time()
                        time_since_first = rospy.get_time() - self.last_first_time
                        print(("integrated", time_since_first))
                        print(("max_first_counter: ", self.max_first_counter))
                        int_displacement, yaw_previous = self.translation_and_yaw(transform_previous)
                        self.pose_msg.pose.position.x = self.x_position_from_state + (int_displacement[0] * altitude)
                        self.pose_msg.pose.position.y = self.y_position_from_state + (int_displacement[1] * altitude)
                        _,_,z,w = tf.transformations.quaternion_from_euler(0,0,yaw_previous)
                        self.pose_msg.pose.orientation.z = z
                        self.pose_msg.pose.orientation.w = w
                        print("Lost the first image !")
                    # if the previous image wasn't visible (the transformation was not
                    # succesful), reset the pose and print lost
                    else:
                        print("Lost!")
                        if self.lost:
                            self.consecutive_lost_counter += 1
                        else:
                            self.lost = True

                self.previous_image = image

        # if the camera is lost over ten times in a row, then publish lost
        # to disable position control
        if self.lost:
            if self.consecutive_lost_counter >= 20:
                self._lostpub.publish(True)
                self.consecutive_lost_counter = 0
        else:
            self.consecutive_lost_counter = 0
            self._lostpub.publish(False)

        # publish the pose message
        self.pose_msg.header.stamp = rospy.Time.now()
        self._posepub.publish(self.pose_msg)

    # normalize image
    def translation_and_yaw(self, transform):
        translation_x_y = [0 - float(transform[0, 2]) / 320,
                            float(transform[1, 2]) / 240]

        # yaw can be up to ~ 20 deg
        yaw_scale = np.sqrt(transform[0, 0]**2 + transform[1, 0]**2)
        yaw_y_x = [float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale]
        yaw = np.arctan2(yaw_y_x[0], yaw_y_x[1])

        return translation_x_y, yaw

    # subscribe /pidrone/reset_transform
    def reset_callback(self, msg):
        """ Reset the current position and orientation """
        print("Resetting Phase")

        # reset position control variables
        self.first = True

        # reset first image vars
        self.first_image_counter = 0
        self.max_first_counter = 0
        self.last_first_time = None

        # reset the pose values
        self.pose_msg = PoseStamped()

        self._lostpub.publish(False)
        print("done")

    # subscribe /pidrone/position_control
    def position_control_callback(self, msg):
        ''' Set whether the pose is calculated and published '''
        self.position_control = msg.data
        print(("Position Control", self.position_control))

    def state_callback(self, msg):
        """
        Store position readings from State
        """
        self.x_position_from_state = msg.pose_with_covariance.pose.position.x
        self.y_position_from_state = msg.pose_with_covariance.pose.position.y
        
    
def main():
    rigid_transform_node = RigidTransformNode("rigid_transform_node")
    rospy.spin()

if __name__ == '__main__':
    main()