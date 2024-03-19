#!/usr/bin/env python3

"""
ROS Node for Monitoring Vehicle Status. This is a simple read multiple topics, convert values to human readable units, and publish to a single custom ROS message that is published at 100Hz.

Purpose is to be used in conjunction with the ACTor GUI and other ACTor nodes that can efficiently use a single high speed message with all the information needed for montoring the vehicle. This significantly reduces Python processing overhead from each node that needs to read multiples of these topics and convert values to human readable units before using them.

This script is part of a larger ROS-based system for controlling and monitoring a vehicle's behavior.

Dependencies:
- ROS (Robot Operating System)
- actor_ros package (for custom ROS messages)
- dbw_polaris_msgs package (for Drive By Wire messages)
- Redis Key Value Store

Authors:
- [Devson Butani] <dbutani@ltu.edu>

License: MIT
"""

import math  # Math Library
import subprocess  # Subprocess Python API

import redis  # Redis Key Value Store Python API
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
from std_msgs.msg import Bool, Header  # ROS Messages

# End of Imports --------------------------------------------------------------


# Start of Callbacks and Functions --------------------------------------------


def report_accelerator_callback(ThrottleReport_msg):
    """Report accelerator percent"""
    global accelerator_percent

    # Get pedal position as a percentage
    accelerator_percent = ThrottleReport_msg.pedal_output * 100
    # NOTE: 20 - 80 percent range limited by the sensor


def report_brakes_callback(BrakeReport_msg):
    """Report brake pedal percent"""
    global brake_percent

    # Get pedal position as a percentage
    brake_percent = (BrakeReport_msg.torque_input / 8000) * 100
    # NOTE: Max torque is 8kNm. input is measured in Nm when pedal is pressed.


def report_steering_callback(SteeringReport_msg):
    """Report steering angle and vehicle speed"""
    global steering_wheel_angle, road_angle, speed

    # Get steering wheel angle and convert to degrees then to road angle in 16.2:1 ratio
    steering_wheel_angle = math.degrees(SteeringReport_msg.steering_wheel_angle)  # Convert radians to degrees
    road_angle = steering_wheel_angle / 16.2  # Convert steering wheel angle to drive wheel angle
    # Get speed in m/s and convert to mph
    speed = SteeringReport_msg.speed * 2.23694  # Convert from m/s to mph


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
    global requested_speed, requested_road_angle, last_twist_time

    last_twist_time = rospy.Time.now()  # Keep track of last message time

    # Get requested speed and road angle
    requested_speed = Twist_msg.linear.x
    requested_road_angle = Twist_msg.angular.z


# NOTE: Not used at the moment
# def speed_limit_callback(TimerEvent):
#     """Report the speed limit"""
#     global speed_limit

#     # Get speed limit
#     speed_limit = rospy.get_param("speed_limit", -1.0)


def enabled_callback(Enabled_msg):
    """Report if ROS control over vehicle is enabled from callback"""
    global enabled

    # Get enable state
    enabled = Enabled_msg.data


def estop_state_callback(msg):
    """Report the estop state"""
    global estop_state

    estop_state = msg.data


def estop_heartbeat_callback(msg):
    """Report the estop heartbeat timestamp"""
    global time_last_heartbeat

    time_last_heartbeat = msg.stamp


def estop_physical_button_callback(msg):
    """Report the estop physical button state"""
    global estop_physical_button

    estop_physical_button = msg.data


def estop_wireless_button_callback(msg):
    """Report the estop wireless button state"""
    global estop_wireless_button

    estop_wireless_button = msg.data


def estop_software_button_callback(msg):
    """Report the estop software button state"""
    global estop_software_button

    estop_software_button = msg.data


def write_to_redis(status_msg):
    """Write statuses to redis server"""
    global redis

    if not redis.ping():  # Check if redis server is connected
        # Write values to redis server
        # NOTE: Values are written to redis server as strings
        redis.set("is_simulated", str(status_msg.is_simulated))
        redis.set("is_autonomous", str(status_msg.is_autonomous))
        redis.set("is_tele_operated", str(status_msg.is_tele_operated))

        redis.set("accelerator_percent", status_msg.accelerator_percent)
        redis.set("brake_percent", status_msg.brake_percent)
        redis.set("steering_wheel_angle", status_msg.steering_wheel_angle)
        redis.set("road_angle", status_msg.road_angle)
        redis.set("gear", status_msg.gear)
        redis.set("speed", status_msg.speed)

        redis.set("is_enabled", str(status_msg.is_enabled))
        redis.set("requested_speed", status_msg.requested_speed)
        redis.set("requested_road_angle", status_msg.requested_road_angle)

        redis.set("estop_state", str(status_msg.estop_state))
        redis.set("estop_heartbeat", str(status_msg.estop_heartbeat))
        redis.set("estop_physical_button", str(status_msg.estop_physical_button))
        redis.set("estop_wireless_button", str(status_msg.estop_wireless_button))
        redis.set("estop_software_button", str(status_msg.estop_software_button))


def publish_status(TimerEvent):
    """Create and publish status message. Rate controlled by ropy.Timer"""
    global accelerator_percent, brake_percent, road_angle, speed, speed_limit, gear
    global requested_speed, requested_road_angle, enabled
    global estop_state, estop_heartbeat, estop_physical_button, estop_wireless_button, estop_software_button
    global time_last_heartbeat, last_twist_time

    # Check if Twist message has timed out - set to 0 if so
    if rospy.Time.now() - last_twist_time > twist_timeout:
        requested_speed = 0
        requested_road_angle = 0

    # Check if received heartbeat is within timeout
    if rospy.Time.now() - time_last_heartbeat > heartbeat_timeout:
        estop_heartbeat = False
    else:
        estop_heartbeat = True

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
    # set to 0 to prevent keeping memory of old values.
    requested_speed = 0
    requested_road_angle = 0

    # E-Stop
    status.estop_state = estop_state
    status.estop_heartbeat = estop_heartbeat
    status.estop_physical_button = estop_physical_button
    status.estop_wireless_button = estop_wireless_button
    status.estop_software_button = estop_software_button

    pub_status.publish(status)
    write_to_redis(status)


# End of Callbacks and Functions ----------------------------------------------


# Start of ROS node -----------------------------------------------------------
rospy.init_node("actor_status")  # Namespace is set in launch file. '/actor' by default


# Start Redis Key-Value Store Server -----
redis_server_command = "redis-server"
redis_server_process = subprocess.Popen(redis_server_command.split())
rospy.sleep(2)  # Wait for redis server to start
redis = redis.Redis(host="localhost", port=6379, db=0, decode_responses=True)  # Connect to redis server
rospy.sleep(2)  # Sleep for 2 seconds before starting


# Initialize global variables in a dictionary -----
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
    "estop_state": False,
    "estop_heartbeat": False,
    "estop_physical_button": False,
    "estop_wireless_button": False,
    "estop_software_button": False,
}

# Make all variables global
globals().update(globals_dict)

# Get one time parameters and topics
is_simulated = rospy.get_param("is_simulated")
topic_status = rospy.get_param("status")
speed_limit = rospy.get_param("speed_limit", -1.0)  # incase control node's dynamic reconfigure is not loaded yet


# Define subscribers -----
rospy.Subscriber(rospy.get_param("enabled"), Bool, enabled_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("estop_state"), Bool, estop_state_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("estop_heartbeat"), Header, estop_heartbeat_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("estop_physical_button"), Bool, estop_physical_button_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("estop_wireless_button"), Bool, estop_wireless_button_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("estop_software_button"), Bool, estop_software_button_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_accelerator"), ThrottleReport, report_accelerator_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_brakes"), BrakeReport, report_brakes_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_steering"), SteeringReport, report_steering_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("report_gear"), GearReport, report_gear_callback, queue_size=1)
rospy.Subscriber(rospy.get_param("drive"), Twist, drive_twist_callback, queue_size=1)
twist_timeout = rospy.Duration(0.1)  # 100ms
last_twist_time = rospy.Time(0)
time_last_heartbeat = rospy.Time(0)
heartbeat_timeout = rospy.Duration(1 / 10)  # 10Hz
# NOTE: Not displaying speed limit at the moment
# rospy.Timer(rospy.Duration(1.0), speed_limit_callback)  # Check speed limit every 1 second

# Status publishers
pub_status = rospy.Publisher(topic_status, ActorStatus, queue_size=10)
rate = 100  # Hz
rospy.Timer(rospy.Duration(1 / rate), publish_status)

rospy.loginfo("actor_status node running.")
try:
    rospy.spin()
except rospy.ROSInterruptException:
    redis.close()
    redis_server_process.terminate()
    pass

# End of ROS node -------------------------------------------------------------
