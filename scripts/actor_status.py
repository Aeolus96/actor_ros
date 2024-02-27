#!/usr/bin/env python3

"""
ROS Node for Monitoring Vehicle Status. This is a simple read multiple topics, convert values to human readable units, and publish to a single custom ROS message that is published at 100Hz.

Purpose is to be used in conjunction with the ACTor GUI and other ACTor nodes that can efficiently use a single high speed message with all the information needed for driving the vehicle. This significantly reduces Python processing overhead from each node that needs to read these topics and convert values to human readable units before using them.

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
from actor_ros.msg import ActorStatus  # Custom ROS Message

# Drive By Wire Messages (Developed by DataSoeed) (https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/)
from dbw_polaris_msgs.msg import (
    BrakeReport,
    GearReport,
    SteeringReport,
    ThrottleReport,
)
from geometry_msgs.msg import Twist  # ROS Messages
from std_msgs.msg import Bool  # ROS Messages

# End of Imports --------------------------------------------------------------


# Start of Callbacks ----------------------------------------------------------


def report_accelerator_callback(ThrottleReport_msg):
    """Report accelerator percent"""
    global accelerator_percent

    # Get pedal position as a percentage
    accelerator_percent = round((ThrottleReport_msg.pedal_output * 100), 2)
    # NOTE: 20 - 80 percent range limited by the sensor


def report_brakes_callback(BrakeReport_msg):
    """Report brake pedal percent"""
    global brake_percent

    # Get pedal position as a percentage
    brake_percent = round(((BrakeReport_msg.torque_input / 8000) * 100), 2)
    # NOTE: Max torque is 8kNm. input is measured in Nm when pedal is pressed.


def report_steering_callback(SteeringReport_msg):
    """Report steering angle and vehicle speed"""
    global steering_wheel_angle, road_angle, speed

    # Get steering wheel angle and convert to degrees then to road angle in 16.2:1 ratio
    steering_wheel_angle = math.degrees(SteeringReport_msg.steering_wheel_angle)  # Convert radians to degrees
    road_angle = round((steering_wheel_angle / 16.2), 2)  # Convert steering wheel angle to drive wheel angle
    # Get speed in m/s and convert to mph
    speed = round(SteeringReport_msg.speed * 2.23694, 2)  # Convert from m/s to mph


def report_gear_callback(GearReport_msg):
    """Report gear"""
    global gear

    gear_dict = {
        0: "NONE",
        1: "PARK",
        2: "REVERSE",
        3: "NEUTRAL",
        4: "DRIVE",
        5: "LOW",
    }

    # Get gear number from GearReport
    gear_number = GearReport_msg.state.gear
    gear = gear_dict[gear_number]  # Convert gear number to string


def drive_twist_callback(Twist_msg):
    """Report the requested twist values from callback"""
    global requested_speed, requested_road_angle

    # TODO: Implement timeout to 0
    # Get requested speed and road angle
    requested_speed = Twist_msg.linear.x
    requested_road_angle = Twist_msg.angular.z


def speed_limit_callback(TimerEvent):
    """Report the speed limit"""
    global speed_limit

    # Get speed limit
    speed_limit = rospy.get_param("speed_limit", -1.0)


def enabled_callback(Enabled_msg):
    """Report if ROS control over vehicle is enabled from callback"""
    global enabled

    # Get enable state
    enabled = Enabled_msg.data
    pass


def publish_status(TimerEvent):
    """Create and publish status message. Rate controlled by ropy.Timer"""
    global accelerator_percent, brake_percent, road_angle, speed, speed_limit, gear
    global requested_speed, requested_road_angle, enabled

    # Create and publish status message
    status = ActorStatus()

    # States
    status.is_simulated = is_simulated
    status.is_autonomous = False  # TODO: Implement this option later
    status.is_tele_operated = False  # TODO: Implement this option later

    # Vehicle States
    status.accelerator_percent = round(accelerator_percent, 3)
    status.brake_percent = round(brake_percent, 3)
    status.steering_wheel_angle = round(steering_wheel_angle, 3)
    status.road_angle = round(road_angle, 3)
    status.gear = gear
    status.speed = round(speed, 2)

    # ROS Control
    status.is_enabled = enabled
    status.speed_limit = round(speed_limit, 3)
    status.requested_speed = round(requested_speed, 3)
    status.requested_road_angle = round(requested_road_angle, 3)

    pub_status.publish(status)


# End of Callbacks ------------------------------------------------------------

# Start of ROS node -----------------------------------------------------------
rospy.init_node("actor_status")  # Namespace is set in launch file. '/actor' by default
rospy.sleep(2)  # Sleep for 2 seconds before starting

# Initialize global variables in a dictionary
globals_dict = {
    "accelerator_percent": 0.0,
    "brake_percent": 0.0,
    "road_angle": 0.0,
    "steering_wheel_angle": 0.0,
    "speed": 0.0,
    "speed_limit": -1.0,
    "gear": "NONE",
    "requested_speed": 0.0,
    "requested_road_angle": 0.0,
    "enabled": False,
}

# Make all variables global
globals().update(globals_dict)


# Get one time parameters and topics
is_simulated = rospy.get_param("is_simulated")
topic_status = rospy.get_param("status")
speed_limit = rospy.get_param("speed_limit", -1.0)  # incase control node's dynamic reconfigure is not loaded yet

# Define subscribers
rospy.Subscriber(rospy.get_param("enabled"), Bool, enabled_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_accelerator"), ThrottleReport, report_accelerator_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_brakes"), BrakeReport, report_brakes_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_steering"), SteeringReport, report_steering_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_gear"), GearReport, report_gear_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("drive"), Twist, drive_twist_callback, queue_size=1)
rospy.Timer(rospy.Duration(1.0), speed_limit_callback)  # Check speed limit every 1 second

# Status publishers
pub_status = rospy.Publisher(topic_status, ActorStatus, queue_size=10)
rate = 100  # Hz
rospy.Timer(rospy.Duration(1 / rate), publish_status)

rospy.loginfo("actor_status node running.")
try:
    rospy.spin()
except rospy.ROSInterruptException:
    pass

# End of ROS node -------------------------------------------------------------
