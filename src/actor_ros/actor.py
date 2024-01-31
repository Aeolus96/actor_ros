# ROS Python API
import rospy

# Drive By Wire Messages (Developed by DataSoeed) (https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/)
from dbw_polaris_msgs.msg import BrakeCmd, Gear, GearCmd, SteeringCmd, ThrottleCmd

# Logging Library (pip install loguru) - logging made simple
from loguru import logger

# Munching Library (pip install munch) - dictionaries but attribute-style access
from munch import munchify

# YAML Library (pip install PyYAML) - YAML parsing
import yaml

# ROS Package API (pip install rospkg) - ROS package information access
import rospkg

# ROS Messages
from std_msgs.msg import Bool, Empty, Float32, Int32, String, UInt8

# Dynamic Reconfigure
# from dynamic_reconfigure.server import Server


class Actor:
    def __init__(self):
        """Initialize the ACTor instance"""

        logger.info("ACTor initializing...")

        # Define attributes
        self.is_simulated = False
        self.is_autonomous = False
        self.is_tele_operated = False
        self.is_enabled = False
        self.steering_angle = 0
        self.brake_percent = 0
        self.accelerator_percent = 0
        self.speed_limit = 5  # mph
        self.gear

        # Define topics
        self.load_topics()

        # Define messages
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
        # TODO: Consider using wheel angle directly
        self.pub_stat_steering_angle = rospy.Publisher(self.topics.status.steering_angle, Float32, queue_size=1)
        self.pub_stat_brake_percent = rospy.Publisher(self.topics.status.brake_percent, UInt8, queue_size=1)
        self.pub_stat_accelerator_percent = rospy.Publisher(self.topics.status.accelerator_percent, UInt8, queue_size=1)
        # TODO: Consider converting from m/s to mph
        self.pub_stat_speed = rospy.Publisher(self.topics.status.speed, Float32, queue_size=1)
        self.pub_stat_is_speed_limit = rospy.Publisher(self.topics.status.speed_limit, UInt8, queue_size=1)
        self.pub_stat_gear = rospy.Publisher(self.topics.status.gear, String, queue_size=1)

        # Define subscribers

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
            except yaml.YAMLError as exc:
                print(exc)

        # Munch the dictionary into an attribute-style object
        config = munchify(yaml_dict)

        # Assign topics as attributes to Actor class
        if self.is_simulated:
            self.topics = config.topics.simulator
        else:
            self.topics = config.topics.real

        self.topics.status = config.topics.status

        print(self.topics.accelerator)
        print(self.topics.status.steering_angle)

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

    def shift_gear(self, gear):
        """Shift gear"""
        # TODO: Implement gear shifting and feedback based speed safety
        self.msg_gear.gear = gear
        self.msg_shift_gear.cmd = self.msg_gear
