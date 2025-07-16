#!/usr/bin/env python3
import traceback
import sys
import yaml
import asyncio

import rospy
import rospkg
import signal
import numpy as np

print("tf import")
import tf
print("done tf import")

import command_values as cmds
from sensor_msgs.msg import Imu
from serial import SerialException
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import Quaternion
from pidrone_pkg.msg import Battery, Mode, RC, State
from sensor_msgs.msg import Range
import os

from unavlib.control import UAVControl
from unavlib.modules.utils import inavutil


class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/fly_commands
    /pidrone/desired/mode
    /pidrone/heartbeat/range
    /pidrone/heartbeat/web_interface
    /pidrone/heartbeat/pid_controller
    /pidrone/state
    """

    def __init__(self):
        # Connect to the flight controller board
        print("initializing uNAVlib")
        self.uav = None  # Will be initialized in connect_to_board
        # stores the current and previous modes
        self.curr_mode = 'DISARMED'         #initialize as disarmed
        self.prev_mode = 'DISARMED'         #initialize as disarmed
        # store the command to send to the flight controller
        self.command = cmds.disarm_cmd      #initialize as disarmed
        self.last_command = cmds.disarm_cmd
        # store the mode publisher
        self.modepub = None
        # store the time for angular velocity calculations
        self.time = rospy.Time.now()
        self.debug_output = False  # Set to True only when debugging
        self.loop_task = None  # Store the asyncio task
        
        # Initialize the Imu Message
        ############################
        header = Header()
        header.frame_id = 'Body'
        header.stamp = rospy.Time.now()

        self.imu_message = Imu()
        self.imu_message.header = header

        # Initialize the Battery Message
        ################################
        self.battery_message = Battery()
        self.battery_message.vbat = None
        self.battery_message.amperage = None
        # Battery configuration
        self.battery_cells = 0  # Auto-detect: 0=unknown, 3=3S, 4=4S
        self.cell_voltage_full = 4.2  # Full charge voltage per cell
        self.cell_voltage_low = 3.7   # Low battery warning threshold per cell
        self.cell_voltage_critical = 3.5  # Critical battery threshold per cell
        
        # Accelerometer parameters
        ##########################
        print("loading")
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/multiwii.yaml" % path) as f:
            means = yaml.load(f, Loader=yaml.SafeLoader)
        print("done")
        self.accRawToMss = 9.8 / means["az"]
        self.accZeroX = means["ax"] * self.accRawToMss
        self.accZeroY = means["ay"] * self.accRawToMss
        self.accZeroZ = means["az"] * self.accRawToMss
        self.quaternion_buffer = np.zeros(4)  # Pre-allocate buffer for quaternion
        
        # Connect to the board
        self.connect_to_board()

    async def connect_to_board(self):
        """Connect to the flight controller board using uNAVlib"""
        try:
            # Try first USB port
            self.uav = UAVControl(device='/dev/ttyACM0', baudrate=115200, platform="MULTICOPTER")
            self.uav.msp_override_channels = [1, 2, 3, 4, 5]  # Enable MSP override for essential channels
            await self.uav.connect()
            print("Connected to flight controller on /dev/ttyACM0")
        except SerialException:
            try:
                # Try second USB port
                self.uav = UAVControl(device='/dev/ttyACM1', baudrate=115200, platform="MULTICOPTER")
                self.uav.msp_override_channels = [1, 2, 3, 4, 5]  # Enable MSP override for essential channels
                await self.uav.connect()
                print("Connected to flight controller on /dev/ttyACM1")
            except SerialException:
                print('\nCannot connect to the flight controller board.')
                print('The USB is unplugged. Please check connection.')
                raise

    # ROS subscriber callback methods:
    ##################################
    def desired_mode_callback(self, msg):
        """ Set the current mode to the desired mode """
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode
        self.update_command()

    def fly_commands_callback(self, msg):
        """ Store and send the flight commands if the current mode is FLYING """
        if self.curr_mode == 'FLYING' or self.curr_mode == 'LAND':
            r = msg.roll
            p = msg.pitch
            y = msg.yaw
            t = msg.throttle
            self.command = [r, p, t, y] + cmds.idle_cmd[4:8]

    # Update methods:
    #################
    async def update_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """
        # Get attitude data
        attitude_data = self.uav.get_attitude()
        if not attitude_data:
            return
            
        # Get IMU data
        imu_data = self.uav.get_imu()
        if not imu_data:
            return
            
        # Calculate values to update imu_message:
        roll = np.deg2rad(attitude_data['roll'])
        pitch = -np.deg2rad(attitude_data['pitch'])  # Negative to match previous convention
        heading = np.deg2rad(attitude_data['heading'])
        
        # Transform heading to standard math conventions
        heading = (-heading) % (2 * np.pi)
        
        # Get the previous roll, pitch, heading values
        previous_quaternion = self.imu_message.orientation
        quaternion_array = [previous_quaternion.x, previous_quaternion.y, previous_quaternion.z, previous_quaternion.w]
        previous_roll, previous_pitch, previous_heading = tf.transformations.euler_from_quaternion(quaternion_array)

        # Convert heading to range [0, 2pi)
        previous_heading = previous_heading % (2 * np.pi)

        # Pre-calculate trig values
        sin_roll = np.sin(roll)
        cos_roll = np.cos(roll)
        sin_pitch = np.sin(pitch)
        cos_pitch = np.cos(pitch)

        # Transform euler angles into quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        self.quaternion_buffer[:] = quaternion  # Copy to pre-allocated buffer
        
        # Calculate the linear accelerations
        lin_acc_x = imu_data['accX'] * self.accRawToMss - self.accZeroX
        lin_acc_y = imu_data['accY'] * self.accRawToMss - self.accZeroY
        lin_acc_z = imu_data['accZ'] * self.accRawToMss - self.accZeroZ

        # Rotate the IMU frame to align with our convention
        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z

        # Account for gravity using pre-calculated trig values
        g = 9.8
        lin_acc_x_drone_body = lin_acc_x_drone_body + g*sin_roll*cos_pitch
        lin_acc_y_drone_body = lin_acc_y_drone_body + g*cos_roll*(-sin_pitch)
        lin_acc_z_drone_body = lin_acc_z_drone_body + g*(1 - cos_roll*cos_pitch)

        # Calculate the angular velocities
        time = rospy.Time.now()
        dt = time.to_sec() - self.time.to_sec()
        if dt > 0:  # Avoid division by zero
            dr = roll - previous_roll
            dp = pitch - previous_pitch
            dh = heading - previous_heading
            angvx = self.near_zero(dr / dt)
            angvy = self.near_zero(dp / dt)
            angvz = self.near_zero(dh / dt)
        else:
            angvx = angvy = angvz = 0
        self.time = time

        # Update the imu_message
        self.imu_message.header.stamp = time
        self.imu_message.orientation.x = quaternion[0]
        self.imu_message.orientation.y = quaternion[1]
        self.imu_message.orientation.z = quaternion[2]
        self.imu_message.orientation.w = quaternion[3]
        self.imu_message.angular_velocity.x = angvx
        self.imu_message.angular_velocity.y = angvy
        self.imu_message.angular_velocity.z = angvz
        self.imu_message.linear_acceleration.x = lin_acc_x_drone_body
        self.imu_message.linear_acceleration.y = lin_acc_y_drone_body
        self.imu_message.linear_acceleration.z = lin_acc_z_drone_body

    async def update_battery_message(self):
        """
        Compute the ROS battery message by reading data from the board.
        Also detect battery type based on voltage.
        """
        # Get analog data using uNAVlib
        analog_data = self.uav.std_send(inavutil.msp.MSP_ANALOG)
        if not analog_data:
            return
            
        # Update battery message
        self.battery_message.vbat = self.uav.board.ANALOG['vbat'] / 10.0  # Convert to volts
        self.battery_message.amperage = self.uav.board.ANALOG['amperage'] / 100.0  # Convert to amps
        
        # Auto-detect battery type if not yet determined
        if self.battery_cells == 0 and self.battery_message.vbat > 0:
            if self.battery_message.vbat > 15.0:  # Likely 4S (16.8V fully charged)
                self.battery_cells = 4
                print("Detected 4S LiPo battery")
            elif self.battery_message.vbat > 10.0:  # Likely 3S (12.6V fully charged)
                self.battery_cells = 3
                print("Detected 3S LiPo battery")
            else:
                # Voltage too low for detection, assume 3S as fallback
                self.battery_cells = 3
                print("Battery voltage too low for detection, assuming 3S LiPo")

    def update_command(self):
        ''' Set command values if the mode is ARMED or DISARMED '''
        if self.curr_mode == 'DISARMED':
            self.command = cmds.disarm_cmd
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd
        # Landing mode is handled by PID controller

    # Helper Methods:
    #################
    def near_zero(self, n):
        """ Set a number to zero if it is below a threshold value """
        return 0 if abs(n) < 0.0001 else n

    def ctrl_c_handler(self, signal, frame):
        """ Initiate landing sequence when ctrl-c is pressed """
        print("\nInitiating landing...")
        # Publish LAND command - PID controller will handle the landing sequence
        self.modepub.publish('LAND')
        # Wait briefly to ensure message is published
        rospy.sleep(0.5)
        print("Landing sequence initiated. Press Ctrl+C again to force exit.")
        # Don't exit immediately to allow landing to start
        # The user can press Ctrl+C again to force exit if needed

    # Heartbeat Callbacks: These update the last time that data was received
    #                       from a node
    def heartbeat_state_estimator_callback(self, msg):
        """Update state_estimator heartbeat"""
        self.heartbeat_state_estimator = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()
        self.range = msg.range

    def shouldIDisarm(self):
        """
        Check if the drone should disarm based on critical conditions.
        Missing heartbeats will trigger warnings but not automatic disarming.
        Low battery will trigger landing sequence instead of immediate disarm.
        """
        curr_time = rospy.Time.now()
        
        # Check all heartbeats and report missing components
        heartbeat_timeout = rospy.Duration.from_sec(1)
        
        # Check for missing components but don't disarm
        if curr_time - self.heartbeat_pid_controller > heartbeat_timeout:
            if not hasattr(self, 'pid_warning_time') or curr_time - self.pid_warning_time > rospy.Duration.from_sec(5):
                print("WARNING: PID controller heartbeat missing")
                self.pid_warning_time = curr_time
        
        if curr_time - self.heartbeat_state_estimator > heartbeat_timeout:
            if not hasattr(self, 'state_warning_time') or curr_time - self.state_warning_time > rospy.Duration.from_sec(5):
                print("WARNING: State estimator heartbeat missing")
                self.state_warning_time = curr_time
        
        if curr_time - self.heartbeat_infrared > heartbeat_timeout:
            if not hasattr(self, 'ir_warning_time') or curr_time - self.ir_warning_time > rospy.Duration.from_sec(5):
                print("WARNING: Infrared sensor heartbeat missing")
                self.ir_warning_time = curr_time
        
        # Only return true for extreme critical conditions
        # For low battery, we'll handle it separately with landing
        return False
        
    def check_battery(self):
        """
        Check battery level and initiate landing if battery is low
        Returns: True if battery is low and landing should be initiated
        """
        if self.battery_cells > 0 and self.battery_message.vbat is not None:
            critical_voltage = self.battery_cells * self.cell_voltage_critical
            if self.battery_message.vbat <= critical_voltage:
                print("\nCRITICAL BATTERY: %.2fV below threshold %.2fV" % (self.battery_message.vbat, critical_voltage))
                return True
        return False

    def wait_for_components(self):
        """
        Wait for all required components to come online
        """
        curr_time = rospy.Time.now()
        heartbeat_timeout = rospy.Duration.from_sec(1)
        
        pid_online = curr_time - self.heartbeat_pid_controller <= heartbeat_timeout
        state_online = curr_time - self.heartbeat_state_estimator <= heartbeat_timeout
        ir_online = curr_time - self.heartbeat_infrared <= heartbeat_timeout
        
        return pid_online, state_online, ir_online
        
    async def send_rc_cmd(self):
        """ Send commands to the flight controller board using uNAVlib """
        assert len(self.command) is 8, "COMMAND HAS WRONG SIZE, expected 8, got "+str(len(self.command))
        
        # Map command values to RC channels - using only essential channels for multicopter
        self.uav.set_rc_channel("roll", self.command[0])      # Roll - channel 1
        self.uav.set_rc_channel("pitch", self.command[1])     # Pitch - channel 2
        self.uav.set_rc_channel("throttle", self.command[2])  # Throttle - channel 3
        self.uav.set_rc_channel("yaw", self.command[3])       # Yaw - channel 4
        self.uav.set_rc_channel("aux1", self.command[4])      # AUX1 (arm/modes) - channel 5
        
        # No need to send aux2, aux3, aux4 as we're only overriding channels 1-5
        
        if (self.command != self.last_command):
            if self.debug_output:
                print('new command sent:', self.command)
            self.last_command = self.command
            
    async def arm_disarm(self):
        """Arm or disarm the drone based on current mode"""
        if self.curr_mode == 'DISARMED':
            self.uav.set_mode("ARM", on=False)
        elif self.curr_mode == 'ARMED' and self.prev_mode == 'DISARMED':
            self.uav.arm_enable_check()
            self.uav.set_mode("ARM", on=True)
            
    async def main_loop(self, imupub, batpub):
        """Main control loop for the flight controller"""
        try:
            while not rospy.is_shutdown():
                # Check if we should disarm due to extreme critical conditions
                if self.shouldIDisarm():
                    print("Critical condition detected. Disarming.")
                    self.curr_mode = 'DISARMED'
                    await self.arm_disarm()
                    self.modepub.publish(self.curr_mode)
                    break
                    
                # Check battery and initiate landing if needed
                if self.check_battery() and self.curr_mode == 'FLYING':
                    print("Low battery detected. Initiating landing sequence.")
                    self.modepub.publish('LAND')
                    
                # update and publish flight controller readings
                await self.update_battery_message()
                await self.update_imu_message()
                imupub.publish(self.imu_message)
                batpub.publish(self.battery_message)

                # update and send the flight commands to the board
                self.update_command()
                await self.send_rc_cmd()
                
                # Handle arming/disarming
                await self.arm_disarm()

                # publish the current mode of the drone
                self.modepub.publish(self.curr_mode)

                # sleep for the remainder of the loop time (60Hz)
                await asyncio.sleep(1/60.0)
                
        except SerialException:
            print('\nCannot connect to the flight controller board.')
            print('The USB is unplugged. Please check connection.')
        except Exception as e:
            print('there was an internal error', e)
            print(traceback.format_exc())
        finally:
            print('Shutdown received')
            print('Sending DISARM command')
            if self.uav and self.uav.connected:
                self.uav.set_mode("ARM", on=False)
                await asyncio.sleep(0.1)  # Give it time to send the command
                self.uav.stop()


async def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    print("init")
    rospy.init_node(node_name)
    print("done")
    
    # create the FlightController object
    fc = FlightController()
    curr_time = rospy.Time.now()
    fc.heartbeat_infrared = curr_time
    fc.range = None
    fc.heartbeat_pid_controller = curr_time
    fc.heartbeat_state_estimator = curr_time

    # Publishers
    ###########
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=True)
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1, tcp_nodelay=False)
    fc.modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    print('Publishing:')
    print('/pidrone/imu')
    print('/pidrone/mode')
    print('/pidrone/battery')

    # Subscribers
    ############
    rospy.Subscriber("/pidrone/desired/mode", Mode, fc.desired_mode_callback)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback)
    # heartbeat subscribers
    rospy.Subscriber("/pidrone/range", Range, fc.heartbeat_infrared_callback)
    rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, fc.heartbeat_pid_controller_callback)
    rospy.Subscriber("/pidrone/state", State, fc.heartbeat_state_estimator_callback)

    signal.signal(signal.SIGINT, fc.ctrl_c_handler)
    
    # Wait for all components to come online before starting
    print("Waiting for all components to come online...")
    while not rospy.is_shutdown():
        pid_online, state_online, ir_online = fc.wait_for_components()
        
        if pid_online and state_online and ir_online:
            print("All components are online. Starting flight controller loop.")
            break
        
        # Print status every 3 seconds
        if not hasattr(fc, 'last_status_time') or rospy.Time.now() - fc.last_status_time > rospy.Duration.from_sec(3):
            fc.last_status_time = rospy.Time.now()
            print("Waiting for: " + 
                  ("PID controller " if not pid_online else "") +
                  ("State estimator " if not state_online else "") +
                  ("IR sensor " if not ir_online else ""))
        
        # Still publish IMU and battery data while waiting
        await fc.update_battery_message()
        await fc.update_imu_message()
        imupub.publish(fc.imu_message)
        batpub.publish(fc.battery_message)
        
        await asyncio.sleep(1/60.0)
    
    # Start the main loop
    await fc.main_loop(imupub, batpub)


if __name__ == '__main__':
    # Run the asyncio event loop
    asyncio.run(main())