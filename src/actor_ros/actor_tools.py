#!/usr/bin/env python3

# import math  # Math Library

import dictdatabase as DDB  # Database API
import rospy  # ROS Python API

# from dbw_polaris_msgs.msg import (  # Drive By Wire Messages
#     BrakeCmd,
#     BrakeReport,
#     Gear,
#     GearCmd,
#     GearReport,
#     SteeringCmd,
#     SteeringReport,
#     ThrottleCmd,
#     ThrottleReport,
# )
# from geometry_msgs.msg import Twist  # ROS Messages
# from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, String, UInt8  # ROS Messages

# End of Imports --------------------------------------------------------------
#
#
#
#
#
# Start of Utilities ----------------------------------------------------------


class ActorStatusReader:
    """ACTor Status Reader subscribes to ACTor status messages and stores the latest values"""

    def __init__(self, read_from_db=False):
        """Initialize the ACTor Status Message Reader"""
        from actor_ros.msg import ActorStatus  # Custom ROS Message

        self.is_simulated = None
        self.is_autonomous = None
        self.is_tele_operated = None
        self.accelerator_percent = 0
        self.brake_percent = 0
        self.steering_wheel_angle = 0
        self.road_angle = 0
        self.speed = 0
        self.gear = None
        self.is_enabled = None
        self.requested_speed = None
        self.requested_road_angle = None

        if read_from_db:  # In case status is read from a database, do not initialize the subscriber.
            # Initialize a JSON dictionary database named "actor_status"
            database_dictionary = {
                "is_simulated": self.is_simulated,
                "is_autonomous": self.is_autonomous,
                "is_tele_operated": self.is_tele_operated,
                "accelerator_percent": self.accelerator_percent,
                "brake_percent": self.brake_percent,
                "steering_wheel_angle": self.steering_wheel_angle,
                "road_angle": self.road_angle,
                "speed": self.speed,
                "gear": self.gear,
                "is_enabled": self.is_enabled,
                "requested_speed": self.requested_speed,
                "requested_road_angle": self.requested_road_angle,
            }
            DDB.at("actor_status").create(database_dictionary, force_overwrite=True)
            # NOTE: database reads need to called manually using the database_callback()

        else:
            # Start a thread to continuously read status messages
            rospy.Subscriber(rospy.get_param("status"), ActorStatus, self.actor_status_callback, queue_size=1)
            # NOTE: topic accesed from the 'actor/' namespace by default

    def actor_status_callback(self, ActorStatus_msg) -> None:
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

    def database_callback(self) -> None:
        """Callback to get the latest ACTor Status values from the dictionary database"""

        # Get dictionary from database "actor_status"
        with DDB.at("actor_status").session as (session, db_dict):
            # States
            self.is_simulated = db_dict["is_simulated"]
            self.is_autonomous = db_dict["is_autonomous"]
            self.is_tele_operated = db_dict["is_tele_operated"]

            # Hardware information
            self.accelerator_percent = db_dict["accelerator_percent"]
            self.brake_percent = db_dict["brake_percent"]
            self.steering_wheel_angle = db_dict["steering_wheel_angle"]
            self.road_angle = db_dict["road_angle"]
            self.speed = db_dict["speed"]
            self.gear = db_dict["gear"]

            # Control information
            self.is_enabled = db_dict["is_enabled"]
            self.requested_speed = db_dict["requested_speed"]
            self.requested_road_angle = db_dict["requested_road_angle"]

    def simulate_for_testing(self) -> None:
        """Simulate variables for testing purposes. Used to test the GUI"""

        self.is_simulated = False
        self.is_autonomous = False
        self.is_tele_operated = False
        self.accelerator_percent = 35.05
        self.brake_percent = 12.23
        self.steering_wheel_angle = -194.4
        self.road_angle = -12.1
        self.speed = 2.34
        self.gear = "DRIVE"
        self.is_enabled = True
        self.requested_speed = 3.45
        self.requested_road_angle = -9.0

    # End of class ------------------------


class EStopManager:
    """Manager for E-Stop functionality.
    E-Stop can be triggered from anywhere.
    Just import this function and use it in your ROS node"""

    def __init__(self):
        """Initialize the E-Stop Manager for any ROS script"""
        # TODO: use direct os calls
        pass

    def publish_e_stop(self):
        pass


# TODO: Script Player

# TODO: Add Classes to wrap functionality of the ACTor
# These can be separate files however, the functionality of directly accessing data is what is needed here
# TODO: Twist Publisher
# TODO: Lane Centering Subscriber
# TODO: Lidar Subscriber
# TODO: GPS Subscriber
# TODO: Yolo Publisher + Subscriber
# TODO: Barrel Detection Subscriber
