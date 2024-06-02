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
        self.waypoints = None  # List of waypoints

        # Import IGVC Parameters from YAML file -------------------------------
        self.package_directory = rospkg.RosPack().get_path("actor_ros")
        file_name = "igvc_params.yaml" if cfg_file_name is None else cfg_file_name
        file_path = self.package_directory + "/cfg/" + file_name
        cfg_file = actor_tools.YAMLReader(file_path)
        cfg_file.read()
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

    def stop_vehicle(
        self,
        using_brakes: bool = False,
        duration: float = 3.0,
        softness: float = 1.5,
        brake_distance: float = None,
        sign_distance: float = None,
    ) -> None:
        """Stop the vehicle by publishing either a zero twist message or a brake pedal command
        and wait for duration seconds"""
        # from geometry_msgs.msg import Twist  # ROS Messages
        import math

        from dbw_polaris_msgs.msg import BrakeCmd, SteeringCmd, ThrottleCmd

        self.print_highlights(f"Stopping Vehicle for {round(duration, 2)}s...")
        start_time = rospy.Time.now()
        rate_hz = 50  # Hz (dbw times out at 10Hz)
        rate = rospy.Rate(rate_hz)

        if using_brakes:  # Send brake pedal command to DBW
            msg = BrakeCmd()
            msg.pedal_cmd_type = 2
            msg.pedal_cmd = 0.0
            msg.enable = True

            if brake_distance is not None:
                brake_target = 0.5

                msg_steering = SteeringCmd()
                msg_steering.enable = True  # Enable Steering, required 'True' for control via ROS
                msg_steering.cmd_type = SteeringCmd.CMD_ANGLE
                msg_steering.steering_wheel_angle_velocity = math.radians(300)  # deg/s -> rad/s
                msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
                msg_steering.clear = False
                msg_steering.ignore = False
                msg_steering.calibrate = False
                msg_steering.quiet = False
                msg_steering.count = 0

                while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                    msg_steering.steering_wheel_angle_cmd = math.radians(self.lane_center())
                    msg.pedal_cmd = min(brake_target, (1 / max(self.lidar_2d() - brake_distance, 0.1)) / 10)
                    self.pub_brakes.publish(msg)
                    self.pub_steering.publish(msg_steering)
                    rate.sleep()

            elif sign_distance is not None:
                brake_target = 0.5

                msg_steering = SteeringCmd()
                msg_steering.enable = True  # Enable Steering, required 'True' for control via ROS
                msg_steering.cmd_type = SteeringCmd.CMD_ANGLE
                msg_steering.steering_wheel_angle_velocity = math.radians(300)  # deg/s -> rad/s
                msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
                msg_steering.clear = False
                msg_steering.ignore = False
                msg_steering.calibrate = False
                msg_steering.quiet = False
                msg_steering.count = 0

                # msg_throttle = ThrottleCmd()
                # msg_throttle.enable = True
                # msg_throttle.pedal_cmd_type = ThrottleCmd.CMD_PEDAL
                # msg_throttle.pedal_cmd = 0.2

                while not rospy.is_shutdown() and not self.lidar_3d(lidar_zone="right", max_distance=sign_distance):
                    msg_steering.steering_wheel_angle_cmd = math.radians(self.lane_center())
                    msg.pedal_cmd = min(
                        brake_target,
                        (1 / max(self.msg_region_right_closest.data - sign_distance, 0.01)) / 2,
                    )

                    # self.pub_throttle.publish(msg_throttle)

                    # msg.pedal_cmd = brake_target
                    self.pub_brakes.publish(msg)
                    self.pub_steering.publish(msg_steering)
                    rate.sleep()

                print("Waiting")
                start_time = rospy.Time.now()

                while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                    msg.pedal_cmd = 0.5
                    self.pub_brakes.publish(msg)
                    rate.sleep()

                print("done waiting")

            else:
                # Reach target pedal value by increasing pedal value for duration seconds
                brake_target = 0.4  # target brake pedal value
                increment = softness * brake_target / rate_hz  # increment fully in 2 seconds

                while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                    msg.pedal_cmd = min(brake_target, msg.pedal_cmd + increment)  # Increase pedal value
                    self.pub_brakes.publish(msg)
                    self.drive(0.0, 0.0)
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

    def drive(
        self,
        speed=0.0,
        speed_kwargs: dict = {},
        angle=0.0,
        angle_kwargs: dict = {},
    ) -> None:
        """Publishes a twist message to drive the vehicle.\n
        It allows optional function-based speed and angle control.\n
        Make sure the function has no arguments or brackets, returns a float.\n
        Specify kwargs using a dictionary. Example: speed_kwargs={'min_speed': 0.0, 'max_speed': 3.0}\n
        Make sure the vehicle is stopped before requesting a direction change."""
        from geometry_msgs.msg import Twist  # ROS Messages

        if callable(speed):  # Use function-based speed control
            speed = speed(**speed_kwargs)
        if callable(angle):  # Use function-based angle control
            angle = angle(**angle_kwargs)

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
        speed_kwargs: dict = {},
        angle=0.0,
        angle_kwargs: dict = {},
        speed_distance: float = None,
        gps_distance: float = None,
        duration: float = None,
        end_function=None,
        end_function_kwargs: dict = {},
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

                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif gps_distance is not None:
            self.print_highlights(f"Driving for {round(gps_distance, 2)}meters...")
            self.update_current_waypoint()
            start_waypoint = self.waypoint
            while not rospy.is_shutdown() and distance_traveled < gps_distance:
                # Calculate distance based on gps coordinates
                self.update_current_waypoint()
                distance_traveled = start_waypoint.distance_to(self.waypoint)
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif duration is not None:  # Use time-based end condition
            self.print_highlights(f"Driving for {round(duration, 2)}seconds...")
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(duration)):
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
                rate.sleep()

        elif callable(end_function):  # Use function-based end condition
            while not rospy.is_shutdown() and not end_function(**end_function_kwargs):
                self.drive(speed, speed_kwargs, angle, angle_kwargs)
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
        min_distance: float = 0.5,
        max_distance: float = 9.9,
        verbose: bool = False,
    ) -> bool:
        """Uses a combination of 2D and 3D LiDAR to determine whether an object exists within the specified zone and range.
        lidar_zone: 'front' and 'rear' are accepted"""

        within_range = False

        if lidar_zone == "front":
            front_3d_adj = self.msg_region_front_closest.data - 1.2
            within_range = (min_distance < front_3d_adj < max_distance) or (
                min_distance < self.msg_front_2d_lidar_closest_object.data < max_distance
            )

        if lidar_zone == "rear":
            within_range = min_distance < self.msg_rear_2d_lidar_closest_object.data < max_distance

        if lidar_zone == "right":
            within_range = min_distance < self.msg_region_right_closest.data < max_distance

        if verbose:
            print(
                f"T3D Distance: {front_3d_adj:.3f}",
                f"F2D Distance: {self.msg_front_2d_lidar_closest_object.data:.3f}",
                f"R2D Distance: {self.msg_rear_2d_lidar_closest_object.data:.3f}",
                f"R3D Distance: {self.msg_region_right_closest.data:.3f}",
                f"Within Range: {within_range}",
            )

        return within_range

    def lidar_2d(
        self,
        lidar_zone: str = "front",
        min_distance: float = 0.5,
        max_distance: float = 9.9,
        verbose: bool = False,
    ) -> float:
        """Output lidar distance"""

        if lidar_zone == "front":
            return self.msg_front_2d_lidar_closest_object.data

        elif lidar_zone == "rear":
            return self.msg_rear_2d_lidar_closest_object.data

    def read_waypoints(self, file_path: str = None, verbose: bool = False) -> list:
        """Reads waypoints from saved YAML file"""
        from actor_ros.actor_tools import YAMLReader

        file = YAMLReader(file_path=file_path)
        file.read()

        waypoint_list = []
        for i in range(len(file.params)):
            waypoint = eval(f"file.params.waypoint{i}")
            waypoint_list.append(Waypoint(lat=waypoint[0].lat, long=waypoint[1].long, heading=waypoint[2].heading))

        if verbose:
            print(waypoint_list)

        return waypoint_list

    def waypoint_in_range(self, goal_waypoint: "Waypoint" = None, radius: float = 3.0) -> bool:
        """Returns True if GPS coordinates are within the specified radius (meters)"""

        if goal_waypoint is None:
            print("Please specify a waypoint")
            return False

        self.update_current_waypoint()  # Update current waypoint position
        distance = self.waypoint.distance_to(goal_waypoint)
        return distance < radius  # or distance is within radius

    def follow_waypoints(self, radius: float = 1.5, gain: float = 1.0, verbose: bool = False) -> float:
        """Returns the angle needed to follow the waypoint trajectory using a list of Waypoints(class). Make sure the list is a defined object and ordered correctly"""

        if self.waypoints is None:
            print("Please specify a list of waypoints")
            return 0

        num_waypoints = len(self.waypoints)

        if num_waypoints > 0:  # If there are waypoints available
            self.update_current_waypoint()  # Update current waypoint position

            # Calculate target angle based on the average relative angle of the first n waypoints
            n = min(10, num_waypoints)
            target_angle_sum = 0
            for i in range(n):
                target_angle_sum += self.waypoint.relative_bearing_with(self.waypoints[i]) * gain

            target_angle = target_angle_sum / n  # Average target angle

            # Check if waypoint is within specified radius
            if self.waypoint_in_range(goal_waypoint=self.waypoints[0], radius=radius):
                if verbose:
                    print("Reached ", self.waypoints[0])
                self.waypoints.pop(0)  # remove waypoint because it has been sufficiently reached

            return target_angle

        else:  # If there are no more waypoints in the list
            if verbose:
                print("no more wayppoints")
            return 0

    # End of Class ----------------------------------------------------------------------------------------------------


class Waypoint:
    """Class for waypoint lat and long operations"""

    def __init__(self, lat: float = None, long: float = None, heading: float = None) -> None:
        """Sets waypoint lat and long (decimal degrees) and current heading (degrees)"""

        if lat is None or long is None:
            print("Please specify lat, long when creating a waypoint")
            return

        self.lat = lat
        self.long = long
        self.heading = heading if heading is not None else 0

    def __str__(self) -> str:
        return f"Waypoint: {self.lat:.6f}, {self.long:.6f}, {self.heading:.3f}"

    def update(self, lat: float, long: float, heading: float) -> None:
        """Sets waypoint lat and long (decimal degrees)"""

        self.lat = lat
        self.long = long
        self.heading = heading

    def distance_to(self, goal: "Waypoint") -> float:
        """Returns Haversine distance between two waypoints in meters"""

        from math import asin, cos, radians, sin, sqrt

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

    def absolute_bearing_with(self, goal: "Waypoint") -> float:
        """Returns absolute bearing between two waypoints in degrees"""

        from math import atan2, cos, degrees, radians, sin

        phi_1 = radians(self.lat)
        lambda_1 = radians(self.long)
        phi_2 = radians(goal.lat)
        lambda_2 = radians(goal.long)
        delta_lambda = lambda_2 - lambda_1
        x = sin(delta_lambda) * cos(phi_2)
        y = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(delta_lambda)

        return (degrees(atan2(x, y)) + 360) % 360  # Normalized to 0-360

    def relative_bearing_with(self, goal: "Waypoint") -> float:
        """Returns the relative bearing from the current heading to the goal waypoint in degrees"""

        relative_bearing = self.heading - self.absolute_bearing_with(goal)  # degrees

        # Normalize to 0 - 360 only when over -180 or 180
        if relative_bearing < -180:
            relative_bearing += 360
        elif relative_bearing > 180:
            relative_bearing -= 360

        return relative_bearing

    # End of Class ----------------------------------------------------------------------------------------------------
