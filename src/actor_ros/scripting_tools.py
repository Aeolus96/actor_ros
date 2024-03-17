#!/usr/bin/env python3

# Imports -------------------------------------------------------------------------------------------------------------
# NOTE: A lot of these are imported to make the script more readable and enable intellisense in the IDE

import rospkg
import rospy
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

import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions
from actor_ros.msg import ActorStatus  # Custom ROS Message

# End of Imports ------------------------------------------------------------------------------------------------------


class ActorScriptTools:
    """Custom Tooling to be used in IGVC related scripts with ACTor vehicle"""

    def __init__(self, cfg_file_name: str = None):
        """Initializes ROS node, makes publishers and subscribers"""

        # Initialize ROS Node -------------------------------------------------
        rospy.init_node("igvc_script")
        self.status = actor_tools.ActorStatusReader()  # Read ACTor Status messages

        # Import IGVC Parameters from YAML file -------------------------------
        self.package_directory = rospkg.RosPack().get_path("actor_ros")
        file_name = "igvc_params.yaml" if cfg_file_name is None else cfg_file_name
        params = actor_tools.YAMLReader(file_path=self.package_directory + "/cfg/" + file_name)

        # Make Publishers -----------------------------------------------------
        for publisher in params.publishers:
            exec(f"from {publisher.msg_file} import {publisher.msg_type}")
            topic = eval(publisher.topic)
            msg = eval(publisher.msg_type)
            temp_publisher = rospy.Publisher(topic, msg, queue_size=1)
            setattr(self, publisher.name, temp_publisher)  # Initialize publishers as instance attributes

        # Make Subscribers ----------------------------------------------------
        for subscriber in params.subscribers:
            exec(f"from {subscriber.msg_file} import {subscriber.msg_type}")
            topic = eval(subscriber.topic)
            msg = eval(subscriber.msg_type)
            name = f"msg_{subscriber.topic.replace('/', '_')}"  # Replace slashes with underscores
            setattr(self, name, msg)  # Initialize the instance attributes with default message objects
            rospy.Subscriber(topic, msg, callback=lambda msg, name=name: setattr(self, name, msg), queue_size=1)

        self.print_highlights("IGVC Tooling Initialized")

    def print_highlights(self, text: str) -> None:
        """Prints text to stdout in a centered "highlight" style format"""
        width = 80
        padding = ((width - len(text)) // 2) - 2
        print(f"{'-' * padding}| {text} |{'-' * padding}")

    def print_title(self, title: str) -> None:
        """Prints text to stdout in a centered "title" style format"""
        width = 80
        padding = (width - len(title)) // 2
        print("=" * width)
        print(f"{' ' * padding}{title.upper()}{' ' * padding}")
        print("=" * width)

    def stop_vehicle(self, using_brakes: bool = False, duration: float = 3.0) -> bool:
        """Stop the vehicle by publishing either a zero twist message or a brake pedal command
        and wait for duration seconds"""

        self.print_highlights(f"Stopping Vehicle for {round(duration, 2)}s...")
        start_time = rospy.Time.now()
        rate_hz = 50  # Hz (dbw times out at 10Hz)
        rate = rospy.Rate(rate_hz)

        if using_brakes:  # Send brake pedal command to DBW
            msg = BrakeCmd()
            msg.pedal_cmd_type = 2
            msg.pedal_cmd = 0.0
            msg.enable = True

            # Reach target pedal value by increasing pedal value for duration seconds
            brake_target = 0.2  # target brake pedal value
            increment = (duration * rate_hz) / brake_target

            while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                msg.pedal_cmd = min(brake_target, msg.pedal_cmd + increment)  # Increase pedal value
                self.pub_brakes.publish(msg)
                rate.sleep()

        else:  # Send zero twist command to twist publisher
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                self.drive(0.0, 0.0)

    def shift_gear(self, gear_input: str) -> bool:
        """Shifts gear using string input. Please stop the vehicle first before calling this method.
        Gears avaiilable: NONE, REVERSE, NEUTRAL, DRIVE."""

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

        if self.status.gear == gear_dict[gear_input]:
            rospy.logdebug(f"Gear is already {gear_input}")
            return True

        # Publish gear shift command ----------------------------------------------
        msg_gear = Gear()
        msg_gear.gear = gear_dict[gear_input]  # Set gear message
        msg_shift_gear = GearCmd()
        msg_shift_gear.cmd = msg_gear  # Make gear shift command
        self.pub_gear.publish(msg_shift_gear)
        rospy.logdebug(f"Gear shifted to {gear_input}")
        return True

    def drive(self, speed=0.0, angle=0.0) -> None:
        """Publishes a twist message to drive the vehicle.
        It allows optional function-based speed and angle control.
        Make sure the vehicle is stopped before requesting a direction change."""

        if callable(speed):  # Use function-based speed control
            speed = speed()
        if callable(angle):  # Use function-based angle control
            angle = angle()

        # Use input speed and current gear to determine if the vehicle should shift gears
        if speed > 0.0:
            if self.status.gear != "DRIVE":
                self.shift_gear("DRIVE")
        elif speed == 0.0:
            if self.status.gear != "NEUTRAL":
                self.shift_gear("NEUTRAL")
        elif speed < 0.0:  # NOTE: Make sure the vehicle is stopped before requesting a direction change
            if self.status.gear != "REVERSE":
                self.shift_gear("REVERSE")

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angle
        self.pub_twist.publish(msg)

    def drive_for(
        self,
        speed=0.0,
        angle=0.0,
        speed_distance=None,
        gps_distance=None,
        duration=None,
        func=None,
    ) -> None:
        """Publishes a twist message to drive the vehicle for a specified duration or distance.
        Use only one conditional argument: speed_distance or gps_distance or duration or func.
        For func-based arguments, make sure to pass a callable object. Lambda functions are only evaluated once."""

        start_time = rospy.Time.now()
        distance_traveled = 0.0  # meters
        rate = rospy.Rate(20)  # Hz (dbw times out at 10Hz)

        if callable(func):  # Use function-based end condition
            while not rospy.is_shutdown() and not func():
                self.drive(speed, angle)
                rate.sleep()
            return

        if duration is not None:  # Use time-based end condition
            self.print_highlights(f"Driving for {round(duration, 2)}seconds...")
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                self.drive(speed, angle)
                rate.sleep()
            return

        if speed_distance is not None:  # Use speed-based distance calculations
            self.print_highlights(f"Driving for {round(speed_distance, 2)}meters...")
            while not rospy.is_shutdown() and distance_traveled < speed_distance:
                # Calculate distance based on current speed (mph->m/s) and time interval (dt)
                distance_traveled += (self.status.speed / 2.237) * (rospy.Time.now() - start_time).to_sec()
                start_time = rospy.Time.now()  # Reset start time for next iteration

                self.drive(speed, angle)
                rate.sleep()

        if gps_distance is not None:
            # TODO: Implement GPS-based distance calculations
            pass

    def lane_center(self, lane=None) -> float:
        """Outputs the road angle needed to center in the lane.
        Used as an input to drive functions angle argument"""

        # TODO: Implement lane centering or value tuning here.
        # Expected output is the angle that the wheels should point at to center in the lane.
        if lane == "left":
            pass
        if lane == "right":
            pass

        return self.msg_lane_center.data  # raw output from /lane_center topic

    # TODO: Add methods from igvc_python but make them more Pythonic a.k.a intuitive and easy to use

    # End of Class ----------------------------------------------------------------------------------------------------
