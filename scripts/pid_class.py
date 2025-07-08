#!/usr/bin/env python

import rospy

# Target height to maintain in meters
TARGET_HEIGHT = 0.65

# PID parameters for height control - tuned for new throttle values
KP = 80.0   # Reduced for smoother control
KI = 0.15   # Reduced to prevent oscillation
KD = 40.0   # Reduced for less aggressive response

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
        
    def step(self, err, time_elapsed):
        # Special handling for throttle controller with improved height control logic
        if self.is_throttle_controller:
            return self.adaptive_height_step(err, time_elapsed)
            
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
        
    def adaptive_height_step(self, err, time_elapsed):
        """
        Advanced throttle control with adaptive behavior for height maintenance
        based on height_control_flight.py's calculate_throttle function
        Simplified to not rely on height change rate calculations
        """
        # Get current height from error (err = TARGET_HEIGHT - current_height)
        current_height = TARGET_HEIGHT - err/100.0  # Convert from cm to m
        
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
            
        print("Height: %.2fm, Target: %.2fm" % (current_height, TARGET_HEIGHT))
        
        # Emergency braking only when too high
        if current_height > TARGET_HEIGHT * 1.75:
            print("EMERGENCY BRAKING: too high!")
            return 1420  # Minimum throttle to quickly descend
            
        # Calculate error
        error = TARGET_HEIGHT - current_height
        
        # Smooth error to reduce sudden changes
        smoothed_error = error * 0.8 + self.previous_height_error * 0.2
        
        # Update integral term with time consideration
        self.height_integral = self.height_integral + error * dt * 0.5
        self.height_integral = max(-0.1, min(self.height_integral, 0.1))  # Limit integral
        
        # Calculate derivative with smoothed error
        derivative = (smoothed_error - self.previous_height_error) / dt
        
        # Calculate PID terms
        p_term = KP * smoothed_error
        i_term = KI * self.height_integral
        d_term = KD * derivative
        
        if current_height > TARGET_HEIGHT:
            # Above target height
            height_diff = current_height - TARGET_HEIGHT
            
            if height_diff > 0.1: 
                base_throttle = 1450  # Set to 1410 for descent
                gain = 0.6
            else: 
                base_throttle = 1450  # Set to 1410 for descent
                gain = 0.7
                
            throttle_adjustment = p_term + i_term + d_term
            throttle = base_throttle + int(throttle_adjustment * gain)
        else:
            # Below target - gentle control
            # Adaptive base throttle based on distance to target
            height_diff = TARGET_HEIGHT - current_height
            
            if height_diff > 0.3:
                # Significantly below target - stronger response
                base_throttle = 1480  # Set to 1480 for takeoff
                gain = 0.9
            elif height_diff > 0.1:
                # Moderately below target
                base_throttle = 1480  # Set to 1480 for takeoff
                gain = 0.8
            elif height_diff > 0.05:
                # Slightly below target
                base_throttle = 1450  # Slightly lower
                gain = 0.7
            else:
                # Very close to target
                base_throttle = 1450  # Between takeoff and descent values
                gain = 0.6
                
            throttle_adjustment = p_term + i_term + d_term
            throttle = base_throttle + int(throttle_adjustment * gain)
        
        # Smoothly limit throttle range
        throttle = max(1380, min(throttle, 1520))  # Narrower range
        
        # Store values for next iteration
        self.previous_height = current_height
        self.previous_height_error = smoothed_error
        
        print("Throttle: %d, Error: %.2f, P: %.2f, I: %.2f, D: %.2f" % (throttle, error, p_term, i_term, d_term))
        
        return throttle


class PID:

    def __init__(self,

                 roll=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 roll_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 pitch=PIDaxis(2.0, 1.0, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-100, 100)),
                 pitch_low=PIDaxis(0.0, 0.5, 0.0, control_range=(1400, 1600), midpoint=1500, i_range=(-150, 150)),

                 yaw=PIDaxis(0.0, 0.0, 0.0),

                 # Kv 2300 motors have midpoint 1300, Kv 2550 motors have midpoint 1250
                 throttle=PIDaxis(1,
                                  0, #0.5/height_factor * battery_factor,
                                  1,
                                  i_range=(-400, 400), control_range=(1100, 1600),
                                  d_range=(-40, 40), midpoint=1300)
                 ):

        self.trim_controller_cap_plane = 0.05
        self.trim_controller_thresh_plane = 0.0001

        self.roll = roll
        self.roll_low = roll_low

        self.pitch = pitch
        self.pitch_low = pitch_low

        self.yaw = yaw

        self.trim_controller_cap_throttle = 5.0
        self.trim_controller_thresh_throttle = 5.0

        self.throttle = throttle
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

        # Use adaptive height control for throttle
        cmd_t = self.throttle.adaptive_height_step(error.z, time_elapsed)
        
        print("%d, %.3f, %.3f, %.3f, %.3f" % (cmd_t, error.z, self.throttle._p, self.throttle._i, self.throttle._d))
        
        # Return commands in order [ROLL, PITCH, THROTTLE, YAW] (changed from ROLL, PITCH, YAW, THROTTLE)
        return [cmd_r, cmd_p, cmd_t, cmd_y]