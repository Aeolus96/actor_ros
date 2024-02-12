#!/usr/bin/env python3

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
    accelerator_percent = ThrottleReport_msg.pedal_output * 100


def report_brakes_callback(BrakeReport_msg):
    """Report brake pedal percent"""
    global brake_percent

    # Get pedal position as a percentage
    brake_percent = BrakeReport_msg.torque_output * 100
    # Max torque is 8kNm


def report_steering_callback(SteeringReport_msg):
    """Report steering angle and vehicle speed"""
    global road_angle, speed

    # Get steering wheel angle and convert to degrees then to road angle in 17:1 ratio
    steering_wheel_angle = SteeringReport_msg.steering_wheel_angle * 180 / math.pi  # Convert radians to degrees
    road_angle = round((steering_wheel_angle / 17), 2)  # Convert steering wheel angle to drive wheel angle

    # Get speed in m/s and convert to mph
    speed = SteeringReport_msg.speed * 2.23694  # Convert from m/s to mph


def report_gear_callback(GearReport_msg):
    """Report gear"""
    global gear

    # Get gear state
    gear = GearReport_msg.state
    # TODO: Verify if state is numeric or Gear msg type (add .gear)
    # If numeric, then use a dict to display string


def drive_twist_callback(Twist_msg):
    """Report the requested twist values from callback"""
    global requested_speed, requested_road_angle

    # Get requested speed and road angle
    requested_speed = Twist_msg.linear.x
    requested_road_angle = Twist_msg.angular.z


def speed_limit_callback(TimerEvent):
    """Report the speed limit"""
    global speed_limit

    # Get speed limit
    speed_limit = rospy.get_param("speed_limit", -1.0)


def enable_callback(Enable_msg):
    """Report if ROS control over vehicle is enabled from callback"""
    global enabled

    # TODO: Verify if Bool or Empty
    # Get enable state
    enabled = Enable_msg.data


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
    status.accelerator_percent = accelerator_percent
    status.brake_percent = brake_percent
    status.road_angle = road_angle
    status.gear = gear
    status.speed = speed
    status.speed_limit = speed_limit

    # ROS Control
    status.is_enabled = enabled
    status.requested_speed = requested_speed
    status.requested_road_angle = requested_road_angle

    pub_status.publish(status)


# End of Callbacks ------------------------------------------------------------

# Start of ROS node -----------------------------------------------------------
rospy.init_node("actor_status")
rospy.sleep(2)  # Sleep for 2 seconds before starting

# Initialize variables
global accelerator_percent, brake_percent, road_angle, speed, speed_limit, gear
global requested_speed, requested_road_angle, enabled
accelerator_percent = 0.0
brake_percent = 0.0
road_angle = 0.0
speed = 0.0
speed_limit = -1.0
gear = "NONE"
requested_speed = 0.0
requested_road_angle = 0.0
enabled = False

# Get one time parameters and topics
is_simulated = rospy.get_param("is_simulated")
topic_status = rospy.get_param("status")
speed_limit = rospy.get_param("speed_limit", -1.0)  # incase control node's dynamic reconfigure is not loaded yet

# Define subscribers
rospy.Subscriber(rospy.get_param("enable"), Bool, enable_callback, queue_size=1)
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
