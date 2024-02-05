# ROS Python API
import rospy

# Math Library
import math

# Munching Library (pip install munch) - dictionaries but attribute-style access
from munch import munchify

# YAML Library (pip install PyYAML) - YAML parsing
import yaml

# ROS Package API (pip install rospkg) - ROS package information access
import rospkg

# ROS Messages
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, String, UInt8
from geometry_msgs.msg import Twist

# Drive By Wire Messages (Developed by DataSoeed) (https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/)
from dbw_polaris_msgs.msg import (
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

# Dynamic Reconfigure
# from dynamic_reconfigure.server import Server


class Actor:
    def __init__(self, is_simulated=False):
        """Initialize the ACTor instance"""

        rospy.loginfo("ACTor initializing...")

        # Define status attributes
        self.is_initialized = False
        self.is_simulated = True if is_simulated else False
        # simulation mode is enabled only via the 'is_simulated' argument at instantiation
        self.is_autonomous = False
        self.is_tele_operated = False
        self.is_enabled = False
        self.steering_angle = 0
        self.brake_percent = 0
        self.accelerator_percent = 0
        self.speed = 0
        self.speed_limit = 5  # mph
        self.gear = None

        # Define topics
        self.load_topics()

        # Define message types
        self.msg_accelerator = ThrottleCmd()
        self.msg_brakes = BrakeCmd()
        self.msg_steering = SteeringCmd()
        self.msg_gear = Gear()
        self.msg_shift_gear = GearCmd()
        self.zero_dbw_messages()

        # Define publishers
        self.pub_accelerator = rospy.Publisher(self.topics.accelerator, ThrottleCmd, queue_size=1)
        self.pub_brakes = rospy.Publisher(self.topics.brakes, BrakeCmd, queue_size=1)
        self.pub_steering = rospy.Publisher(self.topics.steering, SteeringCmd, queue_size=1)
        self.pub_enable_cmd = rospy.Publisher(self.topics.enable, Empty, queue_size=1)
        self.pub_disable_cmd = rospy.Publisher(self.topics.disable, Empty, queue_size=1)
        # Status publishers
        self.pub_stat_is_initialized = rospy.Publisher(self.topics.status.is_initialized, Bool, queue_size=1)
        self.pub_stat_is_simulated = rospy.Publisher(self.topics.status.is_simulated, Bool, queue_size=1)
        self.pub_stat_is_autonomous = rospy.Publisher(self.topics.status.is_autonomous, Bool, queue_size=1)
        self.pub_stat_is_tele_operated = rospy.Publisher(self.topics.status.is_tele_operated, Bool, queue_size=1)
        self.pub_stat_is_enabled = rospy.Publisher(self.topics.status.is_enabled, Bool, queue_size=1)
        self.pub_stat_steering_angle = rospy.Publisher(self.topics.status.steering_angle, Float32, queue_size=1)
        self.pub_stat_brake_percent = rospy.Publisher(self.topics.status.brake_percent, UInt8, queue_size=1)
        self.pub_stat_accelerator_percent = rospy.Publisher(self.topics.status.accelerator_percent, UInt8, queue_size=1)
        self.pub_stat_speed = rospy.Publisher(self.topics.status.speed, Float32, queue_size=1)
        self.pub_stat_is_speed_limit = rospy.Publisher(self.topics.status.speed_limit, UInt8, queue_size=1)
        self.pub_stat_gear = rospy.Publisher(self.topics.status.gear, String, queue_size=1)

        # Define subscribers
        rospy.Subscriber(self.topics.report_accelerator, ThrottleReport, self.report_accelerator_callback)
        rospy.Subscriber(self.topics.report_brake, BrakeReport, self.report_brake_callback)
        rospy.Subscriber(self.topics.report_steering, SteeringReport, self.report_steering_callback)
        rospy.Subscriber(self.topics.report_gear, GearReport, self.report_gear_callback)
        rospy.Subscriber(self.topics.control.enable, Empty, self.enable_callback)
        rospy.Subscriber(self.topics.control.disable, Empty, self.disable_callback)
        rospy.Subscriber(self.topics.control.cmd_vel, Twist, self.drive_twist_callback)

        # Initializiation complete
        self.is_initialized = True
        rospy.loginfo("ACTor initialized!")

    def load_topics(self):
        """Set topics based on simulation or not"""

        # TODO: Test if this works with ROS

        # Get YAML file using rospkg
        r = rospkg.RosPack()
        package_directory = r.get_path("actor_ros")
        yaml_file_path = package_directory + "/cfg/topics.yaml"
        with open(yaml_file_path, "r") as stream:
            try:
                yaml_dict = yaml.safe_load(stream)

                # Munch the dictionary into an attribute-style object
                config = munchify(yaml_dict)

                # Assign topics as attributes to Actor class
                if self.is_simulated:
                    self.topics = config.topics.simulator
                else:
                    self.topics = config.topics.real

                self.topics.status = config.topics.status
                rospy.loginfo(f"Topics loaded successfully from <{yaml_file_path}>")

            except yaml.YAMLError as e:
                rospy.logerr(e)
                rospy.signal_shutdown(f"Could not load the topics from <{yaml_file_path}>")

    def zero_dbw_messages(self):
        """Set all DBW messages to zero or default values"""

        # Accelerator
        self.msg_accelerator.pedal_cmd = 0.0
        self.msg_accelerator.pedal_cmd_type = ThrottleCmd.CMD_PERCENT  # 0.0 to 1.0
        self.msg_accelerator.enable = False  # Enable Throttle, required 'True' for control via ROS
        # Do not use these without completely understanding how they work on the hardware level:
        self.msg_accelerator.clear = False
        self.msg_accelerator.ignore = False
        self.msg_accelerator.count = 0

        # Brake
        self.msg_brakes.pedal_cmd = 0.0
        self.msg_brakes.pedal_cmd_type = BrakeCmd.CMD_PERCENT  # 0.0 to 1.0
        self.msg_brakes.enable = False  # Enable Brake, required 'True' for control via ROS
        # Do not use these without completely understanding how they work on the hardware level:
        self.msg_brakes.clear = False
        self.msg_brakes.ignore = False
        self.msg_brakes.count = 0

        # Steering
        self.msg_steering.steering_wheel_angle_cmd = 0.0  # radians
        # 17:1 steering to road angle ratio and 69 inch Ackerman wheelbase
        self.msg_steering.enable = False  # Enable Steering, required 'True' for control via ROS
        # Do not use these without completely understanding how they work on the hardware level:
        self.msg_steering.cmd_type = SteeringCmd.CMD_ANGLE  # CAUTION: Torque mode disables lateral acceleration limits
        self.msg_steering.steering_wheel_angle_velocity = 0.0  # rad/s
        self.msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
        self.msg_steering.clear = False
        self.msg_steering.ignore = False
        self.msg_steering.calibrate = False
        self.msg_steering.quiet = False
        self.msg_steering.count = 0

        # Gear shift
        self.msg_gear.gear = Gear.NONE
        # NONE=0
        # PARK=1
        # REVERSE=2
        # NEUTRAL=3
        # DRIVE=4
        # LOW=5
        self.msg_shift_gear.cmd = self.msg_gear

        # TODO: Add ULC messages if needed

    def report_accelerator_callback(self, ThrottleReport_msg):
        """Report accelerator percent"""
        # Get pedal position as a percentage
        self.accelerator_percent = ThrottleReport_msg.pedal_output * 100

    def report_brakes_callback(self, BrakeReport_msg):
        """Report brake pedal percent"""
        # Get pedal position as a percentage
        self.brake_percent = BrakeReport_msg.torque_output * 100
        # Max torque is 8kNm

    def report_steering_callback(self, SteeringReport_msg):
        """Report steering angle and vehicle speed"""
        # Get steering wheel angle and convert to degrees then to drive wheel angle in 17:1 ratio
        steering_wheel_angle = SteeringReport_msg.steering_wheel_angle * 180 / math.pi
        road_angle = steering_wheel_angle / 17
        self.steering_angle = round(road_angle, 2)  # Calling road_angle as 'steering_angle' for the sake of simplicity
        # Get speed in m/s and convert to mph
        self.speed = SteeringReport_msg.speed * 2.23694

    def report_gear_callback(self, GearReport_msg):
        """Report gear"""
        self.gear = GearReport_msg.state
        # TODO: Verify if state is numeric or Gear msg type (add .gear)

    def enable(self):
        """Enable vehicle"""
        self.is_enabled = True
        msg = Empty()
        self.pub_enable_cmd.publish(msg)

    def disable(self):
        """Disable vehicle"""
        self.is_enabled = False
        self.zero_dbw_messages()
        # NOTE: Disable is not an emergency stop. It simply disables any ROS control over the vehicle.
        msg = Empty()
        self.pub_disable_cmd.publish(msg)

    def enable_callback(self, Empty_msg):
        """Enable vehicle from callback"""
        self.enable()

    def disable_callback(self, Empty_msg):
        """Disable vehicle from callback"""
        self.disable()

    def drive_twist_callback(self, Twist_msg):
        """Drive vehicle from twist callback"""
        # TODO: Implement throttle, brake, and steering commands
        # TODO: Implement ULC command mode
        pass

    def shift_gear(self, gear):
        """Shift gear"""
        # TODO: Implement gear shifting and feedback based speed safety
        self.msg_gear.gear = gear
        self.msg_shift_gear.cmd = self.msg_gear


# TODO: Add Controller classes?
#           - Proportional Speed Controller
#           - Shift Gear Checker
#           - Come to stop and reverse?
#           - Brake Controller
#           - Proportional Steering Controller
