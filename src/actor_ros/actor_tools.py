#!/usr/bin/env python3

import math  # Math Library

import rospy  # ROS Python API
from dbw_polaris_msgs.msg import (  # Drive By Wire Messages
    BrakeCmd,
    BrakeReport,
    Gear,
    GearCmd,
    GearReport,
    SteeringCmd,
    SteeringReport,
    ThrottleCmd,
    ThrottleReport,
)
from geometry_msgs.msg import Twist  # ROS Messages
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, String, UInt8  # ROS Messages

# End of Imports --------------------------------------------------------------
#
#
#
#
#
# Start of Utilities ----------------------------------------------------------


class ActorStatusReader:
    """ACTor Status Reader subscribes to ACTor status messages and stores the latest values"""

    def __init__(self):
        """Initialize the ACTor Status Message Reader"""
        from actor_ros.msg import ActorStatus  # Custom ROS Message

        self.is_simulated = None
        self.is_autonomous = None
        self.is_tele_operated = None
        self.accelerator_percent = None
        self.brake_percent = None
        self.steering_wheel_angle = None
        self.road_angle = None
        self.speed = None
        self.gear = None
        self.is_enabled = None
        self.requested_speed = None
        self.requested_road_angle = None

        # Start a thread to continuously read status messages
        rospy.Subscriber(rospy.get_param("status"), ActorStatus, self.actor_status_callback, queue_size=1)
        # NOTE: topic accesed from the 'actor/' namespace by default

    def actor_status_callback(self, ActorStatus_msg):
        """Callback to get the latest ACTor Status message"""

        # States
        self.is_simulated = ActorStatus_msg.is_simulated
        self.is_autonomous = ActorStatus_msg.is_autonomous
        self.is_tele_operated = ActorStatus_msg.is_tele_operated

        # Hardware information
        self.accelerator_percent = ActorStatus_msg.accelerator_percent
        self.brake_percent = ActorStatus_msg.brake_percent
        self.steering_wheel_angle = ActorStatus_msg.steering_wheel_angle
        self.road_angle = ActorStatus_msg.road_angle
        self.speed = ActorStatus_msg.speed
        self.gear = ActorStatus_msg.gear

        # Control information
        self.is_enabled = ActorStatus_msg.is_enabled
        self.requested_speed = ActorStatus_msg.requested_speed
        self.requested_road_angle = ActorStatus_msg.requested_road_angle


# End of class ----------------------------------------------------------------
