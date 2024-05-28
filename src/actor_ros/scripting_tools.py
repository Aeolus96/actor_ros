#!/usr/bin/env python3

# Imports -------------------------------------------------------------------------------------------------------------
# NOTE: A lot of these are imported to make the script more readable and enable intellisense in the IDE.
# Add them as needed in individual methods, python will only import what is needed once. Then uses it from the memory.

import rospkg
import rospy

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

# from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, String, UInt8  # ROS Messages
# from geometry_msgs.msg import Twist  # ROS Messages
# import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions
# from actor_ros.msg import ActorStatus  # Custom ROS Message

# End of Imports ------------------------------------------------------------------------------------------------------


class ActorScriptTools:
    """Custom Tooling to be used in IGVC related scripts with ACTor vehicle"""

    def __init__(self, cfg_file_name: str = None):
        """Initializes ROS node, makes publishers and subscribers"""

        import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions

        # Initialize ROS Node -------------------------------------------------
        rospy.init_node("igvc_script")
        self.status = actor_tools.ActorStatusReader()  # Read ACTor Status messages
        # Set Initial waypoint
        self.waypoint = Waypoint(self.status.latitude, self.status.longitude, self.status.heading)

        # Import IGVC Parameters from YAML file -------------------------------
        self.package_directory = rospkg.RosPack().get_path("actor_ros")
        file_name = "igvc_params.yaml" if cfg_file_name is None else cfg_file_name
        file_path = self.package_directory + "/cfg/" + file_name
        cfg_file = actor_tools.YAMLReader(file_path=file_path)
        rospy.sleep(1)  # Wait for parameters to be loaded

        # Make Publishers -----------------------------------------------------
        for publisher in cfg_file.params.publishers:
            exec(f"from {publisher.msg_file} import {publisher.msg_type}")
            topic = str(publisher.topic)
            msg = eval(publisher.msg_type)
            temp_publisher = rospy.Publisher(topic, msg, queue_size=1)
            setattr(self, publisher.name, temp_publisher)  # Initialize publishers as instance attributes

        # Make Subscribers ----------------------------------------------------
        for subscriber in cfg_file.params.subscribers:
            exec(f"from {subscriber.msg_file} import {subscriber.msg_type}")
            topic = str(subscriber.topic)
            msg = eval(subscriber.msg_type)
            msg_instance = msg()
            name = f"msg{subscriber.topic.replace('/', '_')}"  # Replace slashes with underscores
            setattr(self, name, msg_instance)  # Initialize the instance attributes with default message objects
            rospy.Subscriber(topic, msg, callback=self.any_callback, callback_args=name, queue_size=1)

        self.print_highlights("IGVC Tooling Initialized")

    def any_callback(self, msg, name) -> None:
        """Any message callback function"""
        setattr(self, name, msg)

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

    def stop_vehicle(self, using_brakes: bool = False, duration: float = 3.0) -> None:
        """Stop the vehicle by publishing either a zero twist message or a brake pedal command
        and wait for duration seconds"""
        from dbw_polaris_msgs.msg import BrakeCmd

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
            brake_target = 0.4  # target brake pedal value
            increment = 2 * brake_target / rate_hz  # increment fully in 2 seconds

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
        from dbw_polaris_msgs.msg import Gear, GearCmd

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
        """Publishes a twist message to drive the vehicle.\n
        It allows optional function-based speed and angle control.
        Make sure the function has no arguments and returns a float.\n
        Make sure the vehicle is stopped before requesting a direction change."""
        from geometry_msgs.msg import Twist  # ROS Messages

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

    def update_current_waypoint(self) -> None:
        """Updates self Waypoint instance from the current vehicle status"""

        self.waypoint.update(self.status.latitude, self.status.longitude, self.status.heading)

    def drive_for(
        self,
        speed=0.0,
        angle=0.0,
        speed_distance: float = None,
        gps_distance: float = None,
        duration: float = None,
        function=None,  # Make sure it is a callable object without arguments
        *args,
        **kwargs,
    ) -> None:
        """Publishes a twist message to drive the vehicle for a specified duration or distance.\n
        Use ONLY ONE conditional argument: speed_distance or gps_distance or duration or func.\n
        Also supports functions as arguments, make sure to pass a callable object that returns a Bool every call.
        Arguments are passed separately ex: drive_for(function=lidar.detect, zone="front_close", max_distance=5)\n
        Lambda functions are only evaluated once."""

        start_time = rospy.Time.now()
        distance_traveled = 0.0  # meters
        rate = rospy.Rate(20)  # Hz (dbw times out at 10Hz)

        if speed_distance is not None:  # Use speed-based distance calculations
            self.print_highlights(f"Driving for {round(speed_distance, 2)}meters...")
            while not rospy.is_shutdown() and distance_traveled < speed_distance:
                # Calculate distance based on current speed (mph->m/s) and time interval (dt)
                distance_traveled += (self.status.speed / 2.237) * (rospy.Time.now() - start_time).to_sec()
                start_time = rospy.Time.now()  # Reset start time for next iteration

                self.drive(speed, angle)
                rate.sleep()

        elif gps_distance is not None:
            self.print_highlights(f"Driving for {round(gps_distance, 2)}meters...")
            self.update_current_waypoint()
            start_waypoint = self.waypoint
            while not rospy.is_shutdown() and distance_traveled < gps_distance:
                # Calculate distance based on gps coordinates
                self.update_current_waypoint()
                distance_traveled = start_waypoint.distance_to(self.waypoint)
                self.drive(speed, angle)
                rate.sleep()

        elif duration is not None:  # Use time-based end condition
            self.print_highlights(f"Driving for {round(duration, 2)}seconds...")
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                self.drive(speed, angle)
                rate.sleep()

        elif callable(function):  # Use function-based end condition
            while not rospy.is_shutdown() and not function(*args, **kwargs):
                self.drive(speed, angle)
                rate.sleep()

    def lane_center(self, use_blob: bool = True, blob_gain: float = 45.0, lane: str = None) -> float:
        """Outputs the road angle needed to center in the lane.
        Used as an input to drive functions angle argument"""

        # Expected output is the angle that the wheels should point at to center in the lane.

        # Implement lane centering or value tuning here:
        if use_blob:
            return self.msg_blob_cmd.angular.z * blob_gain

        else:
            # if lane == "left":
            #     pass
            # if lane == "right":
            #     pass
            return self.msg_lane_center.data  # raw output from /lane_center topic

    def yolo_look_for(
        self,
        stop_sign: bool = False,
        tire: bool = False,
        person: bool = False,
        pothole: bool = False,
        size: int = 0.0,
    ):
        """Prompts Route-YOLO to look for a specified object.\n
        This runs only once. At least one object class should be set to True.\n
        Returns whether an object was found with a size larger than param 'size'.\n
        Objects Available: stop_sign, tire, person, pothole"""

        from std_msgs.msg import String

        msg_string = String()
        if stop_sign:
            msg_string.data = "stop sign"
        elif tire:
            msg_string.data = "tire"
        elif person:
            msg_string.data = "person"
        elif pothole:
            msg_string.data = "pothole"
        else:
            raise ("Not correct object type")
        self.pub_yolo.publish(msg_string)

        # Check quantity of object found and size of object
        if stop_sign:
            return self.msg_stop_sign_detected.data > 0 and self.msg_stop_sign_size.data > size
        elif tire:
            return self.msg_tire_detected.data > 0 and self.msg_tire_size.data > size
        elif person:
            return self.msg_person_detected.data > 0 and self.msg_person_size.data > size
        elif pothole:
            return self.msg_pothole_detected.data > 0 and self.msg_pothole_size.data > size

    def lidar_3d(
        self,
        lidar_zone: str = "front",
        min_distance: float = 0.1,
        max_distance: float = 9.9,
    ) -> bool:
        """Uses a combination of 2D and 3D LiDAR to determine whether an object exists within the specified zone and range.
        lidar_zone: 'front' and 'rear' are accepted"""

        if lidar_zone == "front":
            front_3d_adj = self.msg_region_front_closest.data - 1.2
            return (min_distance < front_3d_adj < max_distance) or (
                min_distance < self.msg_front_2d_lidar_closest_object < max_distance
            )

        if lidar_zone == "rear":
            return min_distance < self.msg_rear_2d_lidar_closest_object < max_distance

    def lane_change(self, left: bool = False, right: bool = False):  # TODO
        """Changes lane based on left and right inputs"""

        if left:
            pass
        elif right:
            pass
        else:
            pass

    def waypoint_in_range(self, lat: float = None, long: float = None, radius: float = 3.0) -> bool:
        """Returns True if GPS coordinates are within the specified radius (meters)"""

        if lat is None or long is None:
            return False
        self.update_current_waypoint()
        goal_waypoint = Waypoint(lat, long)
        distance = self.waypoint.distance_to(goal_waypoint)
        return distance < radius  # or distance is within radius

    def follow_waypoints(self, waypoints: list = None, radius: float = 3.0) -> float:
        """Returns the angle needed to follow the waypoint trajectory using a list of Waypoints(class). Make sure the list is a defined object and ordered correctly"""

        if waypoints is None:
            print("Please specify a list of waypoints")
            return 0

        if len(waypoints) > 0:  # If there are waypoints available
            self.update_current_waypoint()
            goal_waypoint = waypoints[0]
            target_angle = self.waypoint.relative_bearing_with(goal_waypoint)

            if self.waypoint_in_range(lat=goal_waypoint.lat, long=goal_waypoint.long, radius=radius):
                print("Current ", self.waypoint)
                print("Reached ", goal_waypoint)
                waypoints.pop(0)  # remove waypoint because it has been sufficiently reached

            return target_angle

        else:  # If there are no more waypoints in the list
            return 0

    # TODO: Add methods from igvc_python but make them more Pythonic a.k.a intuitive and easy to use

    # End of Class ----------------------------------------------------------------------------------------------------


class Waypoint:
    """Class for waypoint lat and long operations"""

    def __init__(self, lat: float = None, long: float = None, current_heading: float = None) -> None:
        """Sets waypoint lat and long (decimal degrees) and current heading (degrees)"""

        if lat is None or long is None:
            print("Please specify lat, long when creating a waypoint")
            return

        self.lat = lat
        self.long = long
        self.current_heading = current_heading if current_heading is not None else 0

    def __str__(self) -> str:
        return f"Waypoint: {self.lat:.6f}, {self.long:.6f}, {self.current_heading:.3f}"

    def update(self, lat: float, long: float, current_heading: float) -> None:
        """Sets waypoint lat and long (decimal degrees)"""

        self.lat = lat
        self.long = long
        self.current_heading = current_heading

    def distance_to(self, goal: "Waypoint") -> float:
        """Returns Haversine distance between two waypoints in meters"""

        from math import sin, cos, sqrt, asin, radians

        radius_earth = 6371000  # meters

        phi_1 = radians(self.lat)
        lambda_1 = radians(self.long)
        phi_2 = radians(goal.lat)
        lambda_2 = radians(goal.long)

        delta_lambda = lambda_2 - lambda_1
        delta_phi = phi_2 - phi_1

        # Haversine formula
        a = sin(delta_phi / 2) ** 2 + cos(phi_1) * cos(phi_2) * sin(delta_lambda / 2) ** 2

        return 2 * radius_earth * asin(sqrt(a))

    def bearing_with(self, goal: "Waypoint") -> float:
        """Returns absolute bearing between two waypoints in degrees"""

        from math import sin, cos, atan2, radians, degrees

        phi_1 = radians(self.lat)
        lambda_1 = radians(self.long)
        phi_2 = radians(goal.lat)
        lambda_2 = radians(goal.long)

        delta_lambda = lambda_2 - lambda_1

        x = sin(delta_lambda) * cos(phi_2)
        y = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(delta_lambda)

        return (degrees(atan2(x, y)) + 360) % 360

    def relative_bearing_with(self, goal: "Waypoint") -> float:
        """Returns the relative bearing from the current heading to the goal waypoint in degrees"""

        absolute_bearing = self.bearing_with(goal)
        relative_bearing = (absolute_bearing - self.current_heading + 360) % 360

        return relative_bearing
