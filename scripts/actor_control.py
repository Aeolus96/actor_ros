#!/usr/bin/env python3

"""
ROS Node for Controlling ACTor Vehicle.

This script is part of a larger ROS-based system for controlling and monitoring a vehicle's behavior.

Dependencies:
- ROS (Robot Operating System)
- actor_ros package (for custom ROS messages)
- dbw_polaris_msgs package (for Drive By Wire messages developed by DataSoeed)
              (https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/)

Authors:
- [Devson Butani] <dbutani@ltu.edu>

License: MIT
"""

from threading import Lock  # Thread Locking

import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions
import rospy  # ROS Python API
from actor_ros.cfg import ActorControlConfig  # Dynamic Reconfigure Config File
from dbw_polaris_msgs.msg import (  # Drive By Wire Messages
    BrakeCmd,
    Gear,
    GearCmd,
    SteeringCmd,
    ThrottleCmd,
)
from dynamic_reconfigure.server import Server  # ROS Dynamic Reconfigure
from geometry_msgs.msg import Twist  # ROS Messages
from simple_pid import PID  # PID Controller Library
from std_msgs.msg import Empty  # ROS Messages

# End of Imports --------------------------------------------------------------
#
#
#
#
#
# Start of Callbacks ----------------------------------------------------------


def dyn_rcfg_callback(config, level):
    """Dynamic Reconfigure Callback"""
    global config_, node_initialized

    config_ = config  # Update config global
    rospy.set_param("speed_limit", config_.speed_limit)  # Set speed limit
    if node_initialized:
        init_pid_controllers()
    return config


def drive_twist_callback(Twist_msg):
    """Drive vehicle using twist messages. Expected mph and degrees"""
    global speed_controller, steering_controller
    global last_twist_msg_time, last_twist_msg_time_lock

    # Update last twist message time for timeout check
    with last_twist_msg_time_lock:
        last_twist_msg_time = rospy.Time.now()

    # Get requested speed and road angle from twist inputs
    requested_speed = speed_limiter(
        Twist_msg.linear.x, lower_limit=config_.reverse_speed_limit, upper_limit=config_.speed_limit
    )
    requested_road_angle = road_angle_limiter(Twist_msg.angular.z)

    # Handle gear shifting
    automatic_gear_shifting(requested_speed)

    # Update controller setpoints
    speed_controller.setpoint = round(requested_speed, 2)
    steering_controller.setpoint = round(requested_road_angle, 2)


def publish_control_messages(TimerEvent):
    """Publish all control messages to ROS topics"""
    global steering_controller, speed_controller

    with last_twist_msg_time_lock:  # Check for twist message timeout
        time_since_last_twist_msg = (rospy.Time.now() - last_twist_msg_time).to_sec()

    if time_since_last_twist_msg < config_.twist_timeout:  # ROS control enabled
        speed_controller.auto_mode = True  # Enable PID calculations
        steering_controller.auto_mode = True  # Enable PID calculations
        msg_accelerator, msg_brakes, msg_steering = reset_drive_messages()

        # Calculate PID outputs
        # ---------------------------------------------------------------------
        # Speed Controller -> accelerator pedal and brake pedal values
        speed_controller_output = speed_controller(actor.speed)

        if actor.gear == "REVERSE":  # negative output still uses accelerator pedal
            speed_controller_output *= -1  # convert to positive to feed the accelerator pedal

        if speed_controller_output > 0:  # Accelerate to reach target speed
            msg_accelerator.enable = True
            msg_accelerator.pedal_cmd = speed_controller_output / 100  # Convert to percent scale (0.0 to 1.0)
        elif speed_controller_output < config_.light_braking_threshold:  # Brake lightly to reduce speed faster
            msg_brakes.enable = True
            msg_brakes.pedal_cmd = config_.light_braking_value
        # < else: Simply coast to slow down >

        # ---------------------------------------------------------------------
        # Steering Controller -> steering wheel angle
        steering_controller_output = steering_controller(actor.road_angle)
        p, i, d = steering_controller.components
        rospy.loginfo(f"Steering Controller: p = {p:2.3f}, i = {i:2.3f}, d = {d:2.3f} IN = {actor.road_angle:2.3f}, OUT = {steering_controller_output:2.3f}")
        if abs(steering_controller_output) > config_.steering_deadband:  # deadband removes jitter
            msg_steering.enable = True
            msg_steering.steering_wheel_angle_cmd = steering_controller_output

        # ---------------------------------------------------------------------
        enable_dbw()
        pub_accelerator.publish(msg_accelerator)
        pub_brakes.publish(msg_brakes)
        pub_steering.publish(msg_steering)

    else:  # ROS control disabled
        speed_controller.auto_mode = False  # Disable PID calculations to conserve CPU
        steering_controller.auto_mode = False  # Disable PID calculations to conserve CPU
        # No need to publish any control messages, emergency braking is controlled via an independent node


# End of Callbacks ------------------------------------------------------------
#
#
#
#
#
# Start of Functions ----------------------------------------------------------


def reset_drive_messages():
    """Set all DBW messages to zero or default values"""

    # Accelerator
    msg_accelerator = ThrottleCmd()
    msg_accelerator.pedal_cmd = 0.0
    msg_accelerator.pedal_cmd_type = ThrottleCmd.CMD_PERCENT  # 0.0 to 1.0
    msg_accelerator.enable = False  # Enable Throttle, required 'True' for control via ROS
    # Do not use these without completely understanding how they work on the hardware level:
    msg_accelerator.clear = False
    msg_accelerator.ignore = False
    msg_accelerator.count = 0

    # Brake
    msg_brakes = BrakeCmd()
    msg_brakes.pedal_cmd = 0.0  # NOTE: At 0.50 it is way too much G force in real world.
    msg_brakes.pedal_cmd_type = BrakeCmd.CMD_PERCENT  # 0.0 to 1.0
    msg_brakes.enable = False  # Enable Brake, required 'True' for control via ROS
    # Do not use these without completely understanding how they work on the hardware level:
    msg_brakes.clear = False
    msg_brakes.ignore = False
    msg_brakes.count = 0

    # Steering
    msg_steering = SteeringCmd()
    msg_steering.steering_wheel_angle_cmd = 0.0  # radians (-600deg to 600deg for ACTor)
    # 16.2:1 steering to road angle ratio and 69 inch Ackerman wheelbase
    msg_steering.enable = False  # Enable Steering, required 'True' for control via ROS
    # Do not use these without completely understanding how they work on the hardware level:
    msg_steering.cmd_type = SteeringCmd.CMD_ANGLE  # CAUTION: Torque mode disables lateral acceleration limits
    msg_steering.steering_wheel_angle_velocity = 0.0  # rad/s
    msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
    msg_steering.clear = False
    msg_steering.ignore = False
    msg_steering.calibrate = False
    msg_steering.quiet = False
    msg_steering.count = 0

    return msg_accelerator, msg_brakes, msg_steering


def automatic_gear_shifting(requested_speed) -> None:
    """Automatically shifts gear based on requested speed and current speed"""

    # --------------------------------- at or near 0mph
    if abs(actor.speed) < config_.zero_mph_deadzone:
        #
        # ----------------------------- requested staying stopped
        if abs(requested_speed) < config_.zero_mph_deadzone:
            shift_gear("NEUTRAL")

        # ----------------------------- requested moving forward
        if requested_speed > config_.zero_mph_deadzone:
            shift_gear("DRIVE")

        # ----------------------------- requested moving backward
        if requested_speed < -config_.zero_mph_deadzone:
            shift_gear("REVERSE")

    # --------------------------------- already in motion
    else:
        #
        # ----------------------------- requested stopping OR changing direction of motion (requires stopping)
        if abs(requested_speed) < config_.zero_mph_deadzone or (actor.speed * requested_speed) < 0:
            shift_gear("NEUTRAL")

        # ----------------------------- requested moving in the same direction
        else:
            pass


def shift_gear(gear_input: str) -> bool:
    """Shifts gear using string input"""

    gear_input = gear_input.upper()

    gear_dict = {
        "NONE": 0,
        "PARK": 1,
        "REVERSE": 2,
        "NEUTRAL": 3,
        "DRIVE": 4,
        "LOW": 5,
    }  # Only used for internal error checking for this method

    if gear_input not in gear_dict:
        rospy.logerr(f"Invalid gear: {gear_input}")
        return False

    if actor.gear == gear_dict[gear_input]:
        rospy.logdebug(f"Gear is already {gear_input}")
        return True

    if abs(actor.speed) > config_.zero_mph_deadzone:  # Speed is not near zero
        come_to_stop_with_brakes()  # and then shift gear

    # Publish gear shift command ----------------------------------------------
    msg_gear = Gear()
    msg_shift_gear = GearCmd()
    msg_gear.gear = gear_dict[gear_input]  # Set gear message
    msg_shift_gear.cmd = msg_gear  # Make gear shift command
    pub_gear.publish(msg_shift_gear)
    rospy.logdebug(f"Gear shifted to {gear_input}")
    rospy.sleep(0.5)  # Wait for gear to shift to happen in the hardware
    return True


def come_to_stop_with_brakes():
    """Come to a complete stop using brakes"""
    # Initialize Brake message
    msg_brakes = BrakeCmd()
    msg_brakes.pedal_cmd = 0.0
    msg_brakes.pedal_cmd_type = BrakeCmd.CMD_PERCENT  # 0.0 to 1.0
    msg_brakes.enable = True

    first_time = rospy.Time.now()
    brake_timeout = rospy.Duration(5)  # 5 seconds
    rate = 1 / config_.brake_ramp_hz
    while abs(actor.speed) > config_.zero_mph_deadzone or (rospy.Time.now() - first_time) < brake_timeout:
        # Increase brakes gradually upto max value
        msg_brakes.pedal_cmd += 0.01 if msg_brakes.pedal_cmd < config_.brake_max else config_.brake_max
        pub_brakes.publish(msg_brakes)
        rospy.sleep(rate)


def enable_dbw():
    """Enable vehicle control using ROS messages"""
    msg = Empty()
    pub_enable_cmd.publish(msg)


def init_pid_controllers():
    """Initialize PID controllers"""
    global speed_controller, steering_controller

    speed_controller = PID(
        Kp=config_.speed_kp,
        Ki=config_.speed_ki,
        Kd=config_.speed_kd,
        setpoint=actor.speed,  # current speed
        sample_time=1 / config_.control_rate_hz,  # control rate in seconds
        output_limits=(-100.0, 100.0),
        starting_output=actor.speed # Set starting output to current speed
        # NOTE: Output needs to be converted to Accelerator Percent (0.0 to 1.0) for ThrottleCmd
    )

    steering_controller = PID(
        Kp=config_.steering_kp,
        Ki=config_.steering_ki,
        Kd=config_.steering_kd,
        setpoint=actor.road_angle,  # current angle
        sample_time=1 / config_.control_rate_hz,  # control rate in seconds
        output_limits=(-600, 600),  # degrees (-600deg to 600deg for ACTor steering wheel)
        starting_output=actor.road_angle # Set starting output to current angle
    )
    
    steering_controller.proportional_on_measurement = True
    steering_controller.differential_on_measurement = True

    rospy.loginfo(f"PID Controllers Initialized: {actor.speed}, {actor.road_angle}")
    
    rospy.logdebug(
        "Speed Controller Initialized with P, I, D values: "
        + f"{config_.speed_kp}, {config_.speed_ki}, {config_.speed_kd}"  # PID values
    )
    rospy.logdebug(
        "Steering Controller Initialized with P, I, D values: "
        + f"{config_.steering_kp}, {config_.steering_ki}, {config_.steering_kd}"  # PID values
    )


def speed_limiter(speed, lower_limit=-1, upper_limit=5):
    """Limit speed between lower and upper limits"""

    return lower_limit if speed < lower_limit else upper_limit if speed > upper_limit else speed


def road_angle_limiter(steering, lower_limit=-37.5, upper_limit=37.5):
    """Limit steering between lower and upper limits"""

    return lower_limit if steering < lower_limit else upper_limit if steering > upper_limit else steering


# End of Functions ------------------------------------------------------------
#
#
#
#
#
# Start of ROS node -----------------------------------------------------------

rospy.init_node("actor_control")
rospy.loginfo("actor_control node starting.")

# Initialize global variables
node_initialized = False
last_twist_msg_time = rospy.Time(0)
last_twist_msg_time_lock = Lock()

srv = Server(ActorControlConfig, dyn_rcfg_callback)
# Countdown timer while printing initialization message - waiting for dynamic reconfigure to spin up
for i in range(3, 0, -1):
    rospy.loginfo(f"ACTor Control node Initializing in {i}...")
    rospy.sleep(1)

# Define subscribers
rospy.Subscriber(rospy.get_param("drive"), Twist, drive_twist_callback, queue_size=1)
actor = actor_tools.ActorStatusReader()

# Define publishers
pub_accelerator = rospy.Publisher(rospy.get_param("accelerator"), ThrottleCmd, queue_size=1)
pub_brakes = rospy.Publisher(rospy.get_param("brakes"), BrakeCmd, queue_size=1)
pub_steering = rospy.Publisher(rospy.get_param("steering"), SteeringCmd, queue_size=1)
pub_gear = rospy.Publisher(rospy.get_param("gear"), GearCmd, queue_size=1)
pub_enable_cmd = rospy.Publisher(rospy.get_param("enable"), Empty, queue_size=1)

# Initialize speed and steering controllers
init_pid_controllers()
rospy.Timer(rospy.Duration(1 / config_.control_rate_hz), publish_control_messages)
# NOTE: DBW's Expected publishing rate <= 50Hz (10ms) with a 10Hz (100ms) timeout

node_initialized = True

rospy.loginfo("actor_control node running.")
rospy.spin()

# End of ROS node -------------------------------------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
