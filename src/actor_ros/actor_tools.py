#!/usr/bin/env python3

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
    """ACTor Status Reader subscribes to ACTor status messages and stores the latest values.
    Use read_from_redis=True if running outside a ROS node"""

    def __init__(self, read_from_redis=False, simulate_for_testing=False):
        """Initialize the ACTor Status Message Reader"""

        self.simulated_values = simulate_for_testing

        self.is_simulated = False
        self.is_autonomous = False
        self.is_tele_operated = False
        self.accelerator_percent = 0
        self.brake_percent = 0
        self.steering_wheel_angle = 0
        self.road_angle = 0
        self.speed = 0
        self.gear = "NONE"
        self.is_enabled = False
        self.requested_speed = 0
        self.requested_road_angle = 0
        self.e_stop_state = False
        self.e_stop_heartbeat = False
        self.e_stop_physical_button = False
        self.e_stop_wireless_button = False

        # e_stop/state - Bool
        # e_stop/heartbeat - Header
        # e_stop/physical_button - Bool
        # e_stop/wireless_button - Bool

        # e_stop/trigger - Empty
        # e_stop/reset - Empty

        # Values used only for GUI
        self.gui_gear = self.gear[0]
        self.gui_e_stop_text = "STOP" if self.e_stop_state else "RESET"

        if read_from_redis:  # In case status is read from a database, do not initialize the subscriber.
            # NOTE: updates need to be called manually using the redis_callback() function
            try:
                self.connect_to_redis()
            except ConnectionError as e:
                print(e)
                return e

        else:  # Start a thread to continuously read status messages from ROS
            # NOTE: This requires the instance to be created inside a ROS node

            import rospy  # ROS Python API

            from actor_ros.msg import ActorStatus  # Custom ROS Message

            rospy.Subscriber(rospy.get_param("status"), ActorStatus, self.actor_status_callback, queue_size=1)
            # NOTE: topic accesed from the 'actor/' namespace by default

    def __call__(self):
        """Call method to update the latest ACTor Status values"""
        if self.simulated_values:
            self.simulate_for_testing()
        else:
            self.redis_callback()

    def __del__(self):
        """Cleanup method. Closes Redis connection"""
        if hasattr(self, "redis"):
            self.redis.close()

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

    def connect_to_redis(self) -> bool:
        """Connect to Redis server and read the latest ACTor Status values"""

        import time

        import redis  # Redis Key Value Store Python API

        attempts = 0
        max_attempts = 3
        while attempts < max_attempts:
            try:
                # Attempt to connect to Redis server run from the status node
                # NOTE: Using default settings for Redis. Change if required...
                self.redis = redis.Redis(host="localhost", port=6379, db=0, decode_responses=True)
                time.sleep(2)  # Wait for 2 seconds for status node to start
                self.redis_callback()  # Read values from redis server once
                return True  # Connection successful
            except redis.ConnectionError as e:
                print(f"Failed to connect to Redis server: {e}")
                attempts += 1
                time.sleep(2)  # Wait for 2 seconds before retrying
        print("Failed to connect to Redis server after multiple attempts. Exiting.")
        raise ConnectionError("Failed to connect to Redis server after multiple attempts.")

    @staticmethod
    def to_bool(value: str) -> bool:
        """Converts string value to boolean"""
        if value is None:
            return False
        return value.lower() == "true"

    @staticmethod
    def to_float(value: str) -> float:
        """Converts string value to float"""
        return float(value) if value is not None else 0

    def redis_callback(self) -> None:
        """Callback to get the latest ACTor Status values from Redis server"""

        # Read values from redis server - NOTE: type conversion is needed since Redis stores values as strings
        self.is_simulated = self.to_bool(self.redis.get("is_simulated"))
        self.is_autonomous = self.to_bool(self.redis.get("is_autonomous"))
        self.is_tele_operated = self.to_bool(self.redis.get("is_tele_operated"))
        self.accelerator_percent = self.to_float(self.redis.get("accelerator_percent"))
        self.brake_percent = self.to_float(self.redis.get("brake_percent"))
        self.steering_wheel_angle = self.to_float(self.redis.get("steering_wheel_angle"))
        self.road_angle = self.to_float(self.redis.get("road_angle"))
        self.speed = self.to_float(self.redis.get("speed"))
        self.gear = self.redis.get("gear")
        self.is_enabled = self.to_bool(self.redis.get("is_enabled"))
        self.requested_speed = self.to_float(self.redis.get("requested_speed"))
        self.requested_road_angle = self.to_float(self.redis.get("requested_road_angle"))

    def simulate_for_testing(self) -> None:
        """Simulate variables for testing purposes. Used to test the GUI"""
        import random

        self.is_simulated = False
        self.is_autonomous = False
        self.is_tele_operated = False
        self.accelerator_percent = random.uniform(20, 80)  # 35.05
        self.brake_percent = random.uniform(0, 50)  # 12.23
        self.steering_wheel_angle = random.uniform(-600, 600)  # -194.4
        self.road_angle = random.uniform(-37.5, 37.5)  # -12.1
        self.speed = random.uniform(0, 15)  # 2.34
        self.gear = random.choice(["PARK", "REVERSE", "NEUTRAL", "DRIVE"])  # "DRIVE"
        self.is_enabled = True
        self.requested_speed = random.uniform(0, 15)  # 3.45
        self.requested_road_angle = random.uniform(-37.5, 37.5)  # -9.0

        # NOTE: These values can be randomized if needed

    # End of class ------------------------


class EStopManager:
    """Manager for E-Stop functionality.
    E-Stop can be triggered from anywhere."""

    # e_stop/state - Bool
    # e_stop/physical_button - Bool
    # e_stop/wireless_button - Bool
    # e_stop/trigger - Empty
    # e_stop/reset - Empty
    # e_stop/heartbeat - Header

    def __init__(self):
        """Initialize the E-Stop Manager for any script"""
        self.e_stop_topic = "/actor/e_stop/trigger"
        self.e_stop_reset_topic = "/actor/e_stop/reset"

    def __call__(self) -> bool:
        """Call method to trigger E-Stop using the instance"""
        return self.trigger_e_stop()

    def trigger_e_stop(self) -> bool:
        """Trigger E-Stop by publishing an empty message to the E-Stop topic"""

        return self.send_command(f"rostopic pub {self.e_stop_topic} std_msgs/Empty -1")

    def reset(self):
        """Reset E-Stop by publishing an empty message to the E-Stop reset topic"""

        return self.send_command(f"rostopic pub {self.e_stop_reset_topic} std_msgs/Empty -1")

    def send_command(self, command: str) -> bool:
        """Send a custom command to the E-Stop topic"""
        import os

        try:
            os.system(command)
            print("Command sent successfully")
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False

    # End of class ------------------------


class ScriptPlayer:
    """Executes Python scripts as a subprocess."""

    def __init__(self, directory: str):
        """Initialize the Script Player"""
        self.active_directory = directory  # Full path to the directory
        self.file_list = ""

        self.selected_file = ""  # Name of the selected file with extension
        self.process = None
        self.is_running = False
        self.output_text = ""

    def __del__(self):
        """Cleanup method"""
        self.stop_script()

    def load_files(self):
        """Make a list of files in the active directory"""
        import os

        self.file_list = [file for file in os.listdir(self.active_directory) if file.endswith(".py")]
        self.selected_file = ""
        return "Directory loaded"

    def execute(self):
        """Execute the selected file in a separate process"""
        import subprocess
        from threading import Thread

        if self.is_running:
            return "A script is already running. Please stop it first."

        if self.selected_file == "":
            return "Please select a script to execute"

        # Set the running flag and Clear the output text to allow new output
        self.is_running = True
        self.output_text = ""

        try:
            self.process = subprocess.Popen(
                ["python3", f"{self.active_directory}/{self.selected_file}"],
                shell=True,  # Needed to source ROS using .bashrc
                stdout=subprocess.PIPE,  # Captures print() output # NOTE: ROS logging should be used to maintain logs
                stderr=subprocess.PIPE,  # Captures Raised Errors and Exceptions
                universal_newlines=True,
                text=True,
                bufsize=1,
            )

            # Read output streams in separate threads and combine them into output_text
            Thread(target=self.read_output, args=(self.process.stdout,), daemon=True).start()
            Thread(target=self.read_output, args=(self.process.stderr,), daemon=True).start()
            return "Script finished running"

        except Exception as e:
            self.is_running = False
            return f"Error: {e}"

    def read_output(self, stream):
        """Read output stream and add lines into output_text
        stream is direct input stream from stdout or stderr"""

        for line in stream:
            self.output_text += line

        # When EOF is reached (process is terminated), set the running flag to False
        self.is_running = False

    def stop_script(self, timeout=5.0):
        """Stop the currently running script"""

        if self.process and self.is_running:
            self.process.terminate()  # SIGTERM
            self.process.wait(timeout=timeout)

            if self.process.poll() is None:  # If process is still running
                self.process.terminate()  # Forcefully kill the process - SIGKILL
                self.process.wait()
                self.is_running = False
                return "Script stopped"

            else:
                self.is_running = False
                return "Script stopped"

        else:
            return "No script is currently running"

    # End of class ------------------------


# TODO: Add Classes to wrap functionality of the ACTor
# These can be separate files however, the functionality of directly accessing data is what is needed here
# TODO: Twist Publisher
# TODO: Lane Centering Subscriber
# TODO: Lidar Subscriber
# TODO: GPS Subscriber
# TODO: Yolo Publisher + Subscriber
# TODO: Barrel Detection Subscriber
