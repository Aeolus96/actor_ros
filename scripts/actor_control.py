#!/usr/bin/env python3

"""
ROS Node for Controlling ACTor Vehicle.

This script is part of a larger ROS-based system for controlling and monitoring a vehicle's behavior.

Dependencies:
- ROS (Robot Operating System)
- actor_ros package (for custom ROS messages)
- dbw_polaris_ros package (for Drive By Wire messages developed by DataSoeed)
              (https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/)
- dataspeed_ulc_ros package (for ULC messages developed by DataSoeed)
              (https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/src/master/)

Authors:
- [Devson Butani] <dbutani@ltu.edu>

License: MIT
"""

import math

import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions
import rospy  # ROS Python API
from actor_ros.cfg import ActorControlConfig  # Dynamic Reconfigure Config File
from dataspeed_ulc_msgs.msg import UlcCmd  # Drive By Wire ULC Messages
from dbw_polaris_msgs.msg import SteeringCmd  # Drive By Wire Native Messages
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


def drive_twist_callback(Twist_msg) -> None:
    """Updates the input twist message to buffer"""
    global msg_twist_buffer, last_twist_time

    last_twist_time = rospy.Time.now()  # Keep track of last message time
    msg_twist_buffer = Twist_msg  # Update twist buffer global


def publish_vehicle_controls(TimerEvent) -> None:
    """Publish vehicle controls at a controlled rate"""
    global msg_twist_buffer

    if msg_twist_buffer is None:
        # No new twist message received after timeout
        return

    if rospy.Time.now() - last_twist_time > rospy.Duration(config_.twist_timeout):
        # Twist messages timed out, set twist buffer to None
        msg_twist_buffer = None
        rospy.logwarn("Timed out waiting for twist message")
        return

    enable_dbw()

    # Speed -------------------------------------------------------------------
    requested_speed = speed_limiter(msg_twist_buffer.linear.x)

    # Using ULC speed control
    publish_ulc_speed(requested_speed)

    # Steering ----------------------------------------------------------------
    # Using native steering control
    if config_.twist_uses_road_angle:
        publish_steering(requested_road_angle=msg_twist_buffer.angular.z)
    else:  # Use steering wheel angle
        publish_steering(requested_steering_angle=msg_twist_buffer.angular.z)


# End of Callbacks ------------------------------------------------------------
#
#
#
#
#
# Start of Functions ----------------------------------------------------------


def publish_ulc_speed(speed: float) -> None:
    """Publish requested speed to the vehicle using ULC message."""

    ulc_cmd = UlcCmd()
    ulc_cmd.enable_pedals = True
    ulc_cmd.enable_steering = False  # NOTE: Steering control via ULC is not used here
    ulc_cmd.enable_shifting = True
    ulc_cmd.shift_from_park = True

    ulc_cmd.linear_velocity = speed
    ulc_cmd.linear_accel = 0.0
    ulc_cmd.linear_decel = 0.0
    ulc_cmd.jerk_limit_throttle = 0.0
    ulc_cmd.jerk_limit_brake = 0.0

    ulc_cmd.pedals_mode = 0  # Speed mode
    # ---------------------------------------------------------------------

    pub_ulc.publish(ulc_cmd)


def publish_steering(*, requested_steering_angle: float = None, requested_road_angle: float = None) -> None:
    """Publish requested steering to the vehicle.
    Input can be desired degree road angle (-37 to 37) or steering angle (-600 to 600)"""

    if requested_steering_angle is None and requested_road_angle is None:
        rospy.logerr("publish_steering called with no steering angle provided")
        return

    if requested_road_angle is not None:
        requested_steering_angle = steering_limiter(road_angle=requested_road_angle)

    elif requested_steering_angle is not None:
        requested_steering_angle = steering_limiter(steering_wheel_angle=requested_steering_angle)

    # Make steering message -----------------------------------------------
    msg_steering = SteeringCmd()
    msg_steering.steering_wheel_angle_cmd = math.radians(requested_steering_angle)
    # NOTE: (-600deg to 600deg converted to radians)
    msg_steering.enable = True  # Enable Steering, required 'True' for control via ROS

    # Do NOT use these without completely understanding how they work on the hardware level:
    msg_steering.cmd_type = SteeringCmd.CMD_ANGLE  # CAUTION: Torque mode disables lateral acceleration limits
    # Use angle velocity to control rate. Lock to lock = 1200deg i.e. 300deg/s will be 4secs lock to lock
    msg_steering.steering_wheel_angle_velocity = math.radians(config_.steering_rate_dps)  # deg/s -> rad/s
    msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
    msg_steering.clear = False
    msg_steering.ignore = False
    msg_steering.calibrate = False
    msg_steering.quiet = False
    msg_steering.count = 0
    # ---------------------------------------------------------------------

    pub_steering.publish(msg_steering)


def enable_dbw() -> None:
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
    """Limit steering between lower and upper limits
    Input can be desired degree road angle (-37 to 37) or steering angle (-600 to 600)
    """

    lower_limit = -600
    upper_limit = 600
    out_angle = 0.0

    if road_angle is None and steering_wheel_angle is None:
        rospy.logerr("Steering limiter called with no steering angle provided")
        return 0.0

    if road_angle is not None:
        out_angle = road_angle * 16.2  # 16.2:1 is ACTor steering to road angle ratio

    elif steering_wheel_angle is not None:
        out_angle = steering_wheel_angle

    return lower_limit if out_angle < lower_limit else upper_limit if out_angle > upper_limit else out_angle


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
pub_steering = rospy.Publisher(rospy.get_param("steering"), SteeringCmd, queue_size=1)
pub_enable_cmd = rospy.Publisher(rospy.get_param("enable"), Empty, queue_size=1)
pub_ulc = rospy.Publisher(rospy.get_param("ulc"), UlcCmd, queue_size=1)

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
