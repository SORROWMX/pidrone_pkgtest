#!/usr/bin/env python
import traceback
import sys
import yaml

import rospy
import rospkg
import signal
import numpy as np
import threading
import time

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

# Import uNAVlib instead of MultiWii
from unavlib.control.uavcontrol import UAVControl
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
        print("getboard")
        self.board = self.getBoard()
        print("done")
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
        
        # Define channel mapping for RC commands
        # These are the channel numbers in the command array (0-based index)
        self.RC_ROLL = 0
        self.RC_PITCH = 1
        self.RC_THROTTLE = 2
        self.RC_YAW = 3
        
        # Thread control
        self.running = True
        self.flight_thread = None

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
            means = yaml.load(f)
        print("done")
        self.accRawToMss = 9.8 / means["az"]
        self.accZeroX = means["ax"] * self.accRawToMss
        self.accZeroY = means["ay"] * self.accRawToMss
        self.accZeroZ = means["az"] * self.accRawToMss
        self.quaternion_buffer = np.zeros(4)  # Pre-allocate buffer for quaternion
        
        # Start the flight control thread
        self.flight_thread = threading.Thread(target=self.flight_loop)
        self.flight_thread.daemon = True


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
    def update_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """
        # Get attitude data from uNAVlib
        attitude = self.board.get_attitude()
        if not attitude:
            return
            
        # Calculate values to update imu_message:
        roll = np.deg2rad(attitude['roll'])
        pitch = np.deg2rad(attitude['pitch'])
        heading = np.deg2rad(attitude['yaw'])
        
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
        
        # Get IMU data for acceleration
        imu_data = self.board.get_imu()
        if not imu_data:
            return
            
        # Calculate the linear accelerations - adapt to uNAVlib data format
        # uNAVlib uses 'acc' keys instead of 'accX'
        lin_acc_x = imu_data.get('acc', {}).get('x', 0) * self.accRawToMss - self.accZeroX
        lin_acc_y = imu_data.get('acc', {}).get('y', 0) * self.accRawToMss - self.accZeroY
        lin_acc_z = imu_data.get('acc', {}).get('z', 0) * self.accRawToMss - self.accZeroZ

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

    def update_battery_message(self):
        """
        Compute the ROS battery message by reading data from the board.
        Also detect battery type based on voltage.
        """
        # Get analog data from uNAVlib
        analog_data = self.board.std_send(inavutil.msp.MSP_ANALOG)
        if not analog_data:
            return
            
        # Check if ANALOG is available in the board object
        if hasattr(self.board.board, 'ANALOG'):
            # В uNAVlib напряжение уже хранится в вольтах в поле 'voltage'
            self.battery_message.vbat = self.board.board.ANALOG.get('voltage', 0)
            self.battery_message.amperage = self.board.board.ANALOG.get('amperage', 0)
            
            # Вывод отладочной информации
        else:
            # Fallback if ANALOG is not available
            self.battery_message.vbat = 0
            self.battery_message.amperage = 0
            print("Warning: No ANALOG data available in board object")
        
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
            self.board.set_mode("ARM", on=False)
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
                self.board.arm_enable_check()
                self.board.set_mode("ARM", on=True)
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd
        # Landing mode is handled by PID controller

    # Helper Methods:
    #################
    def getBoard(self):
        """ Connect to the flight controller board """
        try:
            # Create UAVControl object instead of MultiWii
            board = UAVControl('/dev/ttyACM0', 115200, receiver="serial")
            # Connect to the board
            board.connect()
            # Configure the board to accept RC overrides
            board.msp_override_channels = [1, 2, 3, 4, 5]
        except SerialException as e:
            print(("usb0 failed: " + str(e)))
            try:
                board = UAVControl('/dev/ttyACM1', 115200, receiver="serial")
                board.connect()
                # Configure the board to accept RC overrides
                board.msp_override_channels = [1, 2, 3, 4, 5]
            except SerialException:
                print('\nCannot connect to the flight controller board.')
                print('The USB is unplugged. Please check connection.')
                raise
                sys.exit()
        return board

    def send_rc_cmd(self):
        """ Send commands to the flight controller board """
        assert len(self.command) is 8, "COMMAND HAS WRONG SIZE, expected 8, got "+str(len(self.command))
        
        # The UAVControl object doesn't have send_RAW_RC directly, but its internal board does
        try:
            # Send the RC values directly using the board's MSP connection
            if self.board.board.send_RAW_RC(self.command):
                dataHandler = self.board.board.receive_msg()
                self.board.board.process_recv_data(dataHandler)
            
        except Exception as e:
            print(f"Error setting RC channels: {e}")
            traceback.print_exc()
        
        if (self.command != self.last_command):
            if self.debug_output:
                print('new command sent:', self.command)
            self.last_command = self.command

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
        
    def flight_loop(self):
        """
        Main flight control loop that runs in a separate thread
        """
        while self.running:
            try:
                # Send RC commands to the flight controller
                self.send_rc_cmd()
                
                # Sleep briefly to avoid overwhelming the flight controller
                time.sleep(0.02)  # 50Hz update rate
            except Exception as e:
                print(f"Error in flight loop: {e}")
                traceback.print_exc()
                time.sleep(1)  # Sleep longer on error
        
        # Clean up when the loop exits
        try:
            self.board.set_mode("ARM", on=False)
            self.board.disconnect()
        except:
            pass


def main():
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
    
    # set the loop rate (Hz)
    r = rospy.Rate(60)
    
    # Start the flight control thread
    fc.flight_thread.start()
    
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
        fc.update_battery_message()
        fc.update_imu_message()
        imupub.publish(fc.imu_message)
        batpub.publish(fc.battery_message)
        
        r.sleep()
    
    try:
        while not rospy.is_shutdown():
            # Check if we should disarm due to extreme critical conditions
            if fc.shouldIDisarm():
                print("Critical condition detected. Disarming.")
                fc.curr_mode = 'DISARMED'
                fc.command = cmds.disarm_cmd
                fc.board.set_mode("ARM", on=False)
                fc.modepub.publish(fc.curr_mode)
                break
                
            # Check battery and initiate landing if needed
            if fc.check_battery() and fc.curr_mode == 'FLYING':
                print("Low battery detected. Initiating landing sequence.")
                fc.modepub.publish('LAND')
                
            # update and publish flight controller readings
            fc.update_battery_message()
            fc.update_imu_message()
            imupub.publish(fc.imu_message)
            batpub.publish(fc.battery_message)

            # update and send the flight commands to the board
            fc.update_command()

            # publish the current mode of the drone
            fc.modepub.publish(fc.curr_mode)

            # sleep for the remainder of the loop time
            r.sleep()
            
    except SerialException:
        print('\nCannot connect to the flight controller board.')
        print('The USB is unplugged. Please check connection.')
    except Exception as e:
        print('there was an internal error', e)
        print(traceback.format_exc())
    finally:
        print('Shutdown received')
        print('Sending DISARM command')
        fc.running = False
        fc.board.set_mode("ARM", on=False)
        if fc.flight_thread.is_alive():
            fc.flight_thread.join(timeout=1.0)
        fc.board.disconnect()


if __name__ == '__main__':
    main()