#!/usr/bin/env python3

"""
ROS Node for Controlling Vehicle.

This script is part of a larger ROS-based system for controlling and monitoring a vehicle's behavior.

Dependencies:
- ROS (Robot Operating System)
- actor_ros package (for custom ROS messages)
- dbw_polaris_msgs package (for Drive By Wire messages)

Authors:
- [Devson Butani] <dbutani@ltu.edu>

License: MIT
"""

import math  # Math Library

import rospy  # ROS Python API
from actor_ros.cfg import ActorControlConfig  # Dynamic Reconfigure Config File
from actor_ros.msg import ActorStatus  # Custom ROS Message

# Drive By Wire Messages (Developed by DataSoeed) (https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/)
from dbw_polaris_msgs.msg import (
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


# Start of Callbacks ----------------------------------------------------------


def dyn_rcfg_callback(config, level):
    """Dynamic Reconfigure Callback"""

    speed_limit = config["speed_limit"]

    rospy.set_param("speed_limit", speed_limit)
    # matrix_width = config["matrix_width"]
    return config


def actor_status_callback(ActorStatus_msg):
    """Get the latest ACTor Status message"""
    global is_simulated, is_autonomous, is_tele_operated
    global accelerator_percent, brake_percent, road_angle, speed, speed_limit, gear
    global requested_speed, requested_road_angle, is_enabled

    # Get latest status
    is_simulated = ActorStatus_msg.is_simulated
    is_autonomous = ActorStatus_msg.is_autonomous
    is_tele_operated = ActorStatus_msg.is_tele_operated

    accelerator_percent = ActorStatus_msg.accelerator_percent
    brake_percent = ActorStatus_msg.brake_percent
    road_angle = ActorStatus_msg.road_angle
    speed = ActorStatus_msg.speed
    speed_limit = ActorStatus_msg.speed_limit
    gear = ActorStatus_msg.gear

    is_enabled = ActorStatus_msg.is_enabled

    # NOTE: not reading these here
    # requested_speed = ActorStatus_msg.requested_speed
    # requested_road_angle = ActorStatus_msg.requested_road_angle


# End of Callbacks ------------------------------------------------------------


# Start of Functions ----------------------------------------------------------


def shift_gear(gear_input: str, second_try=False):
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

    # Error checking
    if gear_input not in gear_dict:
        rospy.logerr(f"Invalid gear: {gear_input}")
        return False

    if gear == gear_dict[gear_input]:
        rospy.loginfo(f"Gear is already {gear_input}")
        return True

    if abs(speed) > 1:
        if second_try:
            rospy.logerr("Cannot shift gears while vehicle is moving")
            return False

        rospy.logwarn("Cannot shift gear when speed is not near zero. Waiting 2 seconds before retrying...")
        rospy.sleep(2)
        return shift_gear(gear_input, second_try=True)

    # Shift gear
    msg_gear.gear = gear_dict[gear_input]  # Set gear message
    msg_shift_gear.cmd = msg_gear  # Make gear shift command
    pub_gear.publish(msg_shift_gear)
    rospy.loginfo(f"Gear shifted to {gear_input}")
    rospy.sleep(0.5)  # Wait for gear to shift to happen in the hardware

    return True


def zero_dbw_messages():
    """Set all DBW messages to zero or default values"""

    # Accelerator
    msg_accelerator.pedal_cmd = 0.0
    msg_accelerator.pedal_cmd_type = ThrottleCmd.CMD_PERCENT  # 0.0 to 1.0
    msg_accelerator.enable = False  # Enable Throttle, required 'True' for control via ROS
    # Do not use these without completely understanding how they work on the hardware level:
    msg_accelerator.clear = False
    msg_accelerator.ignore = False
    msg_accelerator.count = 0

    # Brake
    msg_brakes.pedal_cmd = 0.0
    msg_brakes.pedal_cmd_type = BrakeCmd.CMD_PERCENT  # 0.0 to 1.0
    msg_brakes.enable = False  # Enable Brake, required 'True' for control via ROS
    # Do not use these without completely understanding how they work on the hardware level:
    msg_brakes.clear = False
    msg_brakes.ignore = False
    msg_brakes.count = 0

    # Steering
    msg_steering.steering_wheel_angle_cmd = 0.0  # radians (-600deg to 600deg for ACTor)
    # 17:1 steering to road angle ratio and 69 inch Ackerman wheelbase
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

    # Gear shift
    msg_gear.gear = Gear.NONE
    msg_shift_gear.cmd = msg_gear  # need to do it this way because cmd is gear message type

    # Add ULC messages if needed, not required for now (all functionality works without ULC messages)


def enable():
    """Enable vehicle control using ROS messages"""
    msg = Empty()
    pub_enable_cmd.publish(msg)


def drive_twist_callback(Twist_msg):
    """Drive vehicle from twist callback"""
    global requested_speed, requested_road_angle
    requested_speed = Twist_msg.linear.x
    requested_road_angle = Twist_msg.angular.z

    # TODO: Implement throttle, brake, and steering commands

    # TODO: Shift to neutral and then reverse if negative speed

    # TODO: Shift to neutral and then drive if positive speed

    # TODO: Stop vehicle and shift to park if no input for 10 seconds

    pass


def speed_controller(requested_speed):
    pass
    # TODO: Add Controller classes?
    #           - Proportional Speed Controller
    #           - Come to stop and reverse?
    #           - Brake Controller
    #           - Proportional Steering Controller


# End of Functions ------------------------------------------------------------


# Start of ROS node -----------------------------------------------------------

rospy.init_node("actor_control")
rospy.loginfo("actor_control node starting.")
srv = Server(ActorControlConfig, dyn_rcfg_callback)

# Define subscribers
rospy.Subscriber(rospy.get_param("drive"), Twist, drive_twist_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("status"), ActorStatus, actor_status_callback, queue_size=1)

# Define publishers
pub_accelerator = rospy.Publisher(rospy.get_param("accelerator"), ThrottleCmd, queue_size=1)
pub_brakes = rospy.Publisher(rospy.get_param("brakes"), BrakeCmd, queue_size=1)
pub_steering = rospy.Publisher(rospy.get_param("steering"), SteeringCmd, queue_size=1)
pub_gear = rospy.Publisher(rospy.get_param("gear"), GearCmd, queue_size=1)
pub_enable_cmd = rospy.Publisher(rospy.get_param("enable"), Empty, queue_size=1)
# NOTE: DBW's Expected publishing rate <= 50Hz (10ms) with a 10Hz (100ms) timeout
# NOTE: This package has closed loop control enabled by default and publishes constantly.

# Define message types
msg_accelerator = ThrottleCmd()
msg_brakes = BrakeCmd()
msg_steering = SteeringCmd()
msg_gear = Gear()
msg_shift_gear = GearCmd()
zero_dbw_messages()

rospy.sleep(5)  # Sleep for 5 seconds before starting anything else

rospy.loginfo("actor_control node running.")
rospy.spin()

# End of ROS node ------------------------------------------------------------
