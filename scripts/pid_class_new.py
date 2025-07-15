#!/usr/bin/env python

import rospy

# PID parameters moved to PID class __init__
# Target height to maintain in meters
# TARGET_HEIGHT = 0.65
# KP = 80.0
# KI = 0.15
# KD = 60.0
# HOVER_THROTTLE = 1300  
# DEADBAND = 50          

#####################################################
#						PID							#
#####################################################
class PIDaxis():
    def __init__(self, kp, ki, kd, i_range=None, d_range=None, control_range=(1000, 2000), midpoint=1500, smoothing=True):
        # Tuning
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Config
        self.i_range = i_range
        self.d_range = d_range
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = smoothing
        # Internal
        self.reset()
        # Additional parameters for advanced height control
        self.previous_height = 0
        self.height_integral = 0
        self.previous_height_error = 0
        # Initialize last_time to None, will be set on first step call
        self.last_time = None
        self.is_throttle_controller = False
        # Landing mode flag
        self.landing_mode = False
        self.hover_throttle = 1300  # Default value, will be overridden in PID class
        self.deadband = 50          # Default value, will be overridden in PID class
        self.target_vel_z = 0.0       
        self.current_vel_z = 0.0
        # Add target_height attribute with default value
        self.target_height = 0.65  # Default value, will be overridden in PID class
        
    def reset(self):
        self._old_err = None
        self._p = 0
        self._i = 0
        self._d = 0
        self._dd = 0
        self._ddd = 0
        # Reset height control variables too
        self.previous_height = 0
        self.height_integral = 0
        self.previous_height_error = 0
        # Reset last_time to None
        self.last_time = None
        # Reset landing mode
        self.landing_mode = False
        # Reset velocity variables
        self.target_vel_z = 0.0
        self.current_vel_z = 0.0
        
    def step(self, err, time_elapsed):
        # Special handling for throttle controller with improved height control logic
        if self.is_throttle_controller:
            if self.landing_mode:
                current_height = -err/100.0  # Convert from cm to m
                desired_height = 0  
            else:
                current_height = self.target_height - err/100.0  # Convert from cm to m
                desired_height = self.target_height
                
            return self.barometer_height_step(desired_height, current_height, time_elapsed)
            
        # Standard PID logic for other axes
        if self._old_err is None:
            # First time around prevent d term spike
            self._old_err = err

        # Find the p component
        self._p = err * self.kp

        # Find the i component
        self._i += err * self.ki * time_elapsed
        if self.i_range is not None:
            self._i = max(self.i_range[0], min(self._i, self.i_range[1]))

        # Find the d component
        self._d = (err - self._old_err) * self.kd / time_elapsed
        if self.d_range is not None:
            self._d = max(self.d_range[0], min(self._d, self.d_range[1]))
        self._old_err = err

        # Smooth over the last three d terms
        if self.smoothing:
            self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0)/15.0
            self._ddd = self._dd
            self._dd = self._d

        # Calculate control output
        raw_output = self._p + self._i + self._d
        output = min(max(raw_output + self.midpoint, self.control_range[0]), self.control_range[1])

        return output
   
    def barometer_height_step(self, desired_height, current_height, time_elapsed):
       
        height_error = desired_height - current_height
        
        # Calculate time elapsed since last call
        current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.01  # Default small value for first iteration
        else:
            dt = (current_time - self.last_time).to_sec()
            
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.01  # Safeguard against zero division
        
        if self.previous_height != 0:
            self.current_vel_z = (current_height - self.previous_height) / dt
        self.previous_height = current_height
        
        if not self.landing_mode:
            print("Height: %.2fm, Target: %.2fm, Error: %.2fm, Vel_Z: %.2fm/s" % 
                  (current_height, desired_height, height_error, self.current_vel_z))
        
        # Emergency braking only when too high and not in landing mode
        if current_height > desired_height * 1.75 and not self.landing_mode:
            print("EMERGENCY BRAKING: too high!")
            return 1100  # Minimum throttle to quickly descend
        
        p_term = height_error * self.kp
        
        self.height_integral += height_error * time_elapsed
        self.height_integral = max(-0.2, min(self.height_integral, 0.2))
        i_term = self.height_integral * self.ki
        
        if self.previous_height_error is None:
            d_term = 0
        else:
            d_term = (height_error - self.previous_height_error) / time_elapsed * self.kd
        
        self.previous_height_error = height_error
        
        self.target_vel_z = p_term + i_term + d_term
        
        # Special handling for landing mode
        if self.landing_mode:
            # Gentler descent during landing
            base_throttle = 1220  # Lower base throttle for landing (adjusted for 1300 hover)
            gain = 0.5  # Reduced gain for smoother landing
            
            # When very close to the ground, reduce throttle further
            if current_height < 0.15:
                base_throttle = 1180
                gain = 0.4
                
            throttle_adjustment = p_term + i_term + d_term
            throttle = base_throttle + int(throttle_adjustment * gain)
            
            # Limit throttle range during landing
            throttle = max(1100, min(throttle, 1250))
        else:
            pid_output = p_term + i_term + d_term
            
            if abs(pid_output) < 0.10: 
                throttle = self.hover_throttle
            else:
                scaled_output = int(pid_output * 150) 
                if scaled_output > 0:
                    throttle = self.hover_throttle + self.deadband + scaled_output
                else:
                    throttle = self.hover_throttle - self.deadband + scaled_output
        
        throttle = max(1100, min(throttle, 1500))  
        
        if not self.landing_mode or int(rospy.get_time() * 2) != int((rospy.get_time() - 0.1) * 2):
            print("Throttle: %d (hover: %d, deadband: %d)" % (throttle, self.hover_throttle, self.deadband))
        
        return throttle


class PID:

    def __init__(self,
                 # Default PID parameters that were previously at the top of the file
                 target_height=0.65,
                 kp=0.7,      # Changed from 80.0/100
                 ki=0.0015,   # Changed from 0.15/100
                 kd=0.65,      # Changed from 60.0/100
                 hover_throttle=1300,
                 deadband=30,

                 roll=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 roll_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 pitch=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 pitch_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 yaw=PIDaxis(0.0, 0.0, 0.0),

                 # Throttle controller uses barometer_height_step for altitude control
                 throttle=None
                 ):

        # Store the target height as instance variable
        self.target_height = target_height
        
        self.trim_controller_cap_plane = 0.05
        self.trim_controller_thresh_plane = 0.0001

        self.roll = roll
        self.roll_low = roll_low

        self.pitch = pitch
        self.pitch_low = pitch_low

        self.yaw = yaw

        # Create throttle controller if not provided
        if throttle is None:
            self.throttle = PIDaxis(kp, ki, kd, i_range=(-400, 400), 
                                  control_range=(1100, 1500), d_range=(-40, 40), 
                                  midpoint=hover_throttle)
        else:
            self.throttle = throttle
            
        # Set hover throttle and deadband values
        self.throttle.hover_throttle = hover_throttle
        self.throttle.deadband = deadband
        # Set target height in throttle controller
        self.throttle.target_height = target_height
            
        # Mark throttle controller for special handling
        self.throttle.is_throttle_controller = True

        self._t = None

        # Tuning values specific to each drone
        self.roll_low.init_i = 0.0
        self.pitch_low.init_i = 0.0
        self.reset()

    def reset(self):
        """ Reset each pid and restore the initial i terms """
        # reset time variable
        self._t = None

        # reset individual PIDs
        self.roll.reset()
        self.roll_low.reset()
        self.pitch.reset()
        self.pitch_low.reset()
        self.throttle.reset()

        # restore tuning values
        self.roll_low._i = self.roll_low.init_i
        self.pitch_low._i = self.pitch_low.init_i

    def step(self, error, cmd_yaw_velocity=0):
        """ Compute the control variables from the error using the step methods
        of each axis pid.
        """
        # First time around prevent time spike
        if self._t is None:
            time_elapsed = 1
        else:
            time_elapsed = rospy.get_time() - self._t

        self._t = rospy.get_time()

        # Compute roll command
        ######################
        # if the x velocity error is within the threshold
        if abs(error.x) < self.trim_controller_thresh_plane:
            # pass the high rate i term off to the low rate pid
            self.roll_low._i += self.roll._i
            self.roll._i = 0
            # set the roll value to just the output of the low rate pid
            cmd_r = self.roll_low.step(error.x, time_elapsed)
        else:
            if error.x > self.trim_controller_cap_plane:
                self.roll_low.step(self.trim_controller_cap_plane, time_elapsed)
            elif error.x < -self.trim_controller_cap_plane:
                self.roll_low.step(-self.trim_controller_cap_plane, time_elapsed)
            else:
                self.roll_low.step(error.x, time_elapsed)

            cmd_r = self.roll_low._i + self.roll.step(error.x, time_elapsed)

        # Compute pitch command
        #######################
        if abs(error.y) < self.trim_controller_thresh_plane:
            self.pitch_low._i += self.pitch._i
            self.pitch._i = 0
            cmd_p = self.pitch_low.step(error.y, time_elapsed)
        else:
            if error.y > self.trim_controller_cap_plane:
                self.pitch_low.step(self.trim_controller_cap_plane, time_elapsed)
            elif error.y < -self.trim_controller_cap_plane:
                self.pitch_low.step(-self.trim_controller_cap_plane, time_elapsed)
            else:
                self.pitch_low.step(error.y, time_elapsed)

            cmd_p = self.pitch_low._i + self.pitch.step(error.y, time_elapsed)

        # Compute yaw command
        cmd_y = 1500 + cmd_yaw_velocity

        # Use barometer-based height control for throttle
        cmd_t = self.throttle.step(error.z, time_elapsed)
        
        
        # Return commands in order [ROLL, PITCH, THROTTLE, YAW] (changed from ROLL, PITCH, YAW, THROTTLE)
        return [cmd_r, cmd_p, cmd_t, cmd_y]
        
    def set_landing_mode(self, is_landing):
        """Set landing mode flag for throttle controller"""
        self.throttle.landing_mode = is_landing