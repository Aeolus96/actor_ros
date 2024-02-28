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

import math
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
    global config_, dyn_rcfg_initialized

    dyn_rcfg_initialized = True
    config_ = config  # Update config global
    rospy.set_param("speed_limit", config_.speed_limit)  # Set speed limit
    return config


def drive_twist_callback(Twist_msg):
    """Updates the input twist message to buffer"""
    global msg_twist_buffer, last_twist_time

    last_twist_time = rospy.Time.now()  # Keep track of last message time
    msg_twist_buffer = Twist_msg  # Update twist buffer global


def publish_vehicle_controls(TimerEvent):
    """Publish vehicle controls at a controlled rate"""
    global msg_twist_buffer

    if msg_twist_buffer is None:
        # No new twist message received after timeout
        return

    if rospy.Time.now() - last_twist_time > rospy.Duration(config_.twist_timeout):
        # Twist messages timed out, set twist buffer to None
        msg_twist_buffer = None
        return

    enable_dbw()

    # Speed -------------------------------------------------------------------
    requested_speed = speed_limiter(msg_twist_buffer.linear.x)
    shift_gear_automatically(requested_speed)  # Handle gear shifting first
    # NOTE: if direction of speed is changed, braking to a stop will be handled while shifting gears

    accelerator, brakes = calculate_pedals(requested_speed)
    publish_accelerator(pedal_percent=accelerator / 100.0)
    publish_brakes(pedal_percent=brakes / 100.0)

    # Steering ----------------------------------------------------------------
    if config_.twist_uses_road_angle:
        publish_steering(requested_road_angle=msg_twist_buffer.angular.z)
    else:  # Use steering wheel angle
        publish_steering(requested_steering_angle=msg_twist_buffer.angular.z)

    # Tuning Mode -------------------------------------------------------------
    if config_.tuning_mode:
        rospy.loginfo(f"A = {accelerator:8.2f}, B = {brakes:8.2f}, Diff = {(requested_speed - actor.speed):8.2f}")
        # NOTE: output="screen" is required in the launch file


# End of Callbacks ------------------------------------------------------------
#
#
#
#
#
# Start of Functions ----------------------------------------------------------


def reset_drive_messages():
    """Set all DBW messages to zero or default values and return them.
    Not used at the moment, but can be used in the future."""

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
    # NOTE: Output is the target steering angle in radians not an angle increment
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


def shift_gear_automatically(requested_speed) -> None:
    """Automatically shifts gear based on requested speed and current speed"""

    # --------------------------------- at or near 0mph
    if abs(actor.speed) < config_.speed_deadband:
        #
        # ----------------------------- requested staying stopped
        if abs(requested_speed) < config_.speed_deadband:
            shift_gear("NEUTRAL")

        # ----------------------------- requested moving forward
        if requested_speed > config_.speed_deadband:
            shift_gear("DRIVE")

        # ----------------------------- requested moving backward
        if requested_speed < -config_.speed_deadband:
            shift_gear("REVERSE")

    # --------------------------------- already in motion
    else:
        #
        # ----------------------------- requested stopping OR changing direction of motion (requires stopping)
        if abs(requested_speed) < config_.speed_deadband or (actor.speed * requested_speed) < 0:
            shift_gear("NEUTRAL")

        # else: ----------------------- requested moving in the same direction


def shift_gear(gear_input: str) -> bool:
    """Shifts gear using string input. Returns True if successful
    Gears avaiilable: NONE, PARK, REVERSE, NEUTRAL, DRIVE, LOW"""

    gear_input = gear_input.upper()

    gear_dict = {
        "NONE": 0,
        "PARK": 1,
        "REVERSE": 2,
        "NEUTRAL": 3,
        "DRIVE": 4,
        "LOW": 5,
    }  # Only used for internal error checking for this method
    # NOTE: ACTor only has "NONE", "REVERSE", "NEUTRAL", "DRIVE".

    if gear_input not in gear_dict:
        rospy.logerr(f"Invalid gear: {gear_input}")
        return False

    if actor.gear == gear_dict[gear_input]:
        rospy.logdebug(f"Gear is already {gear_input}")
        return True

    if abs(actor.speed) > config_.speed_deadband:  # Speed is not near zero
        stop_with_brakes()  # and then shift gear

    # Publish gear shift command ----------------------------------------------
    msg_gear = Gear()
    msg_shift_gear = GearCmd()
    msg_gear.gear = gear_dict[gear_input]  # Set gear message
    msg_shift_gear.cmd = msg_gear  # Make gear shift command
    pub_gear.publish(msg_shift_gear)
    rospy.logdebug(f"Gear shifted to {gear_input}")
    return True


def calculate_pedals(requested_speed: float):
    """Calculate pedal values to maintain the vehicle at the requested speed."""

    accelerator = 0
    brakes = 0

    if requested_speed < abs(config_.speed_deadband):  # Requested Stopping the vehicle
        stop_with_brakes()
        return accelerator, brakes

    speed_difference = requested_speed - actor.speed

    if abs(actor.speed) > config_.speed_deadband and abs(speed_difference) > config_.speed_deadband:
        # Speed is not near zero and speed difference is also not near zero

        if speed_difference > config_.acceleration_threshold:
            # Need to Accelerate
            accelerator = actor.accelerator_percent + config_.speed_ramp_up_constant * speed_difference
            brakes = 0

        elif speed_difference < config_.light_braking_threshold:
            # Need to Decelerate
            accelerator = 0
            brakes = min(
                config_.light_braking_max,
                actor.brake_percent + config_.speed_ramp_down_constant * abs(speed_difference),
            )

        else:
            # Need to Coast
            accelerator = 0
            brakes = 0

    return accelerator, brakes


def publish_accelerator(*, pedal_percent: float):
    """Publish requested accelerator pedal value to the vehicle.
    Input is 0.0 to 1.0 however 0.2 is physical minimum and 0.8 is physical maximum"""

    pedal_percent = max(0.0, min(1.0, pedal_percent))

    if 0.2 < pedal_percent > 0.8:  # 0.2 is physical minimum and 0.8 is physical maximum
        # Accept values however no need to publish out of physical range
        # Make Accelerator message --------------------------------------------
        msg_accelerator = ThrottleCmd()
        msg_accelerator.pedal_cmd = pedal_percent
        msg_accelerator.pedal_cmd_type = ThrottleCmd.CMD_PERCENT  # 0.0 to 1.0
        msg_accelerator.enable = False  # Enable Throttle, required 'True' for control via ROS

        # Do not use these without completely understanding how they work on the hardware level:
        msg_accelerator.clear = False
        msg_accelerator.ignore = False
        msg_accelerator.count = 0
        # ---------------------------------------------------------------------

        pub_accelerator.publish(msg_accelerator)


def publish_brakes(*, pedal_percent: float):
    """Publish requested brake pedal value to the vehicle. Input is 0.0 to 1.0 however 0.50 is too much G force"""

    # Make sure it is within range
    brake_maximum = config_.brake_max if actor.speed > 1.0 else 0.3
    # NOTE: 0.3 ~ 30% is more than enough for immediate braking at very low speeds
    pedal_percent = max(0.0, min(brake_maximum, pedal_percent))

    # Make Brake message ------------------------------------------------------
    msg_brakes = BrakeCmd()
    msg_brakes.pedal_cmd = pedal_percent  # NOTE: At 0.50 it is way too much G force in real world.
    msg_brakes.pedal_cmd_type = BrakeCmd.CMD_PERCENT  # 0.0 to 1.0
    msg_brakes.enable = False  # Enable Brake, required 'True' for control via ROS

    # Do not use these without completely understanding how they work on the hardware level:
    msg_brakes.clear = False
    msg_brakes.ignore = False
    msg_brakes.count = 0
    # -------------------------------------------------------------------------

    pub_brakes.publish(msg_brakes)


def stop_with_brakes():
    """Come to a complete stop using brakes. Ramps using brake_ramp_hz in dynamic reconfig"""

    first_time = rospy.Time.now()
    brake_timeout = rospy.Duration(5)  # seconds
    rate = rospy.Rate(config_.brake_ramp_hz)
    brake_value = 0.0
    while abs(actor.speed) > config_.speed_deadband or (rospy.Time.now() - first_time) < brake_timeout:
        # While not nearly stopped or within timeout, increase brakes gradually upto max value
        brake_value += 0.01 if brake_value < config_.brake_max else config_.brake_max
        publish_brakes(pedal_percent=brake_value)
        rate.sleep()


def publish_steering(*, requested_steering_angle: float = None, requested_road_angle: float = None):
    """Publish requested steering to the vehicle.
    Input can be desired degree road angle (-37 to 37) or steering angle (-600 to 600)"""

    requested_steering_angle = (steering_limiter(road_angle=requested_road_angle)) or steering_limiter(
        steering_wheel_angle=requested_steering_angle
    )

    difference = requested_steering_angle - actor.steering_wheel_angle

    if abs(difference) > config_.steering_deadband:  # Avoid large steering angle difference
        # Make steering message -----------------------------------------------
        msg_steering = SteeringCmd()
        msg_steering.steering_wheel_angle_cmd = math.radians(requested_steering_angle)
        # radians (-600deg to 600deg for ACTor)
        # NOTE: Output is the target steering angle in radians not an angle increment
        # 16.2:1 steering to road angle ratio and 69 inch Ackerman wheelbase
        msg_steering.enable = True  # Enable Steering, required 'True' for control via ROS

        # Do NOT use these without completely understanding how they work on the hardware level:
        msg_steering.cmd_type = SteeringCmd.CMD_ANGLE  # CAUTION: Torque mode disables lateral acceleration limits
        msg_steering.steering_wheel_angle_velocity = 0.0  # rad/s
        msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
        msg_steering.clear = False
        msg_steering.ignore = False
        msg_steering.calibrate = False
        msg_steering.quiet = False
        msg_steering.count = 0
        # ---------------------------------------------------------------------

        pub_steering.publish(msg_steering)


def enable_dbw():
    """Enable vehicle control using ROS messages"""
    msg = Empty()
    pub_enable_cmd.publish(msg)


def speed_limiter(speed) -> float:
    """Limit speed between lower and upper limits.
    Unit ambiguous, specify limits in dynamic reconfigure accordingly"""

    lower_limit = config_.reverse_speed_limit
    upper_limit = config_.speed_limit
    return lower_limit if speed < lower_limit else upper_limit if speed > upper_limit else speed


def steering_limiter(*, steering_wheel_angle: float = None, road_angle: float = None) -> float:
    """Limit steering between lower and upper limits"""

    angle = (road_angle * 16.2) or steering_wheel_angle  # 16.2:1 is ACTor steering to road angle ratio
    lower_limit = -600
    upper_limit = 600
    return lower_limit if angle < lower_limit else upper_limit if angle > upper_limit else angle


# End of Functions ------------------------------------------------------------
#
#
#
#
#
# Start of ROS node -----------------------------------------------------------

rospy.init_node("actor_control")
rospy.loginfo("actor_control node starting...")

# Initialize global variables
dyn_rcfg_initialized = False
requested_road_angle = 0
requested_speed = 0

srv = Server(ActorControlConfig, dyn_rcfg_callback)
# waiting for dynamic reconfigure to spin up
while not rospy.is_shutdown() and not dyn_rcfg_initialized:
    rospy.loginfo("Waiting for dynamic reconfigure to spin up...")
    rospy.sleep(1)

# Define subscribers
rospy.Subscriber(rospy.get_param("drive"), Twist, drive_twist_callback, queue_size=1)
msg_twist_buffer = None  # Used for twist messages slower than control rate.
last_twist_time = rospy.Time(0)
rospy.Timer(rospy.Duration(1 / config_.control_rate_hz), publish_vehicle_controls)
# NOTE: DBW's Optimal publishing rate >= 50Hz (10ms) with a (100ms) timeout
actor = actor_tools.ActorStatusReader()

# Define publishers
pub_accelerator = rospy.Publisher(rospy.get_param("accelerator"), ThrottleCmd, queue_size=1)
pub_brakes = rospy.Publisher(rospy.get_param("brakes"), BrakeCmd, queue_size=1)
pub_steering = rospy.Publisher(rospy.get_param("steering"), SteeringCmd, queue_size=1)
pub_gear = rospy.Publisher(rospy.get_param("gear"), GearCmd, queue_size=1)
pub_enable_cmd = rospy.Publisher(rospy.get_param("enable"), Empty, queue_size=1)

rospy.loginfo("actor_control node running.")

try:
    rospy.spin()
except rospy.ROSInterruptException:
    rospy.loginfo("actor_control node shutting down.")
    pass

# End of ROS node -------------------------------------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
