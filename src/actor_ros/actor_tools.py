# Math Library
import math

import rospy  # ROS Python API

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
from geometry_msgs.msg import Twist

# ROS Messages
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, String, UInt8


# Master Class for ACTor Control
class Actor:
    def __init__(self, is_simulated=False, topics=None):
        """Initialize the ACTor instance"""

        rospy.loginfo("ACTor initializing...")

        # Define status attributes (meant for internal write and external read only)
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

        # Control Requests
        self.closed_loop_control = True
        self.requested_speed = 0.0
        self.requested_road_angle = 0.0

        # Define topics (Munchified dictionary that can be accessed as attribute-style object)
        if topics is None:
            rospy.signal_shutdown("Please specify the topics to be used by ACTor")

        self.topics = topics

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
        self.pub_gear = rospy.Publisher(self.topics.gear, GearCmd, queue_size=1)
        self.pub_enable_cmd = rospy.Publisher(self.topics.enable, Empty, queue_size=1)
        self.pub_disable_cmd = rospy.Publisher(self.topics.disable, Empty, queue_size=1)
        # NOTE: DBW's Expected publishing rate <= 50Hz (10ms) with a 10Hz (100ms) timeout
        # NOTE: This package has closed loop control enabled by default and publishes constantly.

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
        rospy.Subscriber(self.topics.report_accelerator, ThrottleReport, self.report_accelerator_callback, queue_size=1)
        rospy.Subscriber(self.topics.report_brake, BrakeReport, self.report_brake_callback, queue_size=1)
        rospy.Subscriber(self.topics.report_steering, SteeringReport, self.report_steering_callback, queue_size=1)
        rospy.Subscriber(self.topics.report_gear, GearReport, self.report_gear_callback, queue_size=1)
        # NOTE: Enable/Disable is not an EMERGENCY STOP. External control removed because not needed at the moment
        # NOTE: Simply publishing a twist message automatically enables/disables the vehicle
        # rospy.Subscriber(self.topics.control.enable, Empty, self.enable_callback, queue_size=1)
        # rospy.Subscriber(self.topics.control.disable, Empty, self.disable_callback, queue_size=1)
        rospy.Subscriber(self.topics.control.cmd_vel, Twist, self.drive_twist_callback, queue_size=1)

        # Initializiation complete
        self.is_initialized = True
        rospy.loginfo("ACTor initialized!")

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
        self.msg_steering.steering_wheel_angle_cmd = 0.0  # radians (-600deg to 600deg for ACTor)
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
        self.msg_shift_gear.cmd = self.msg_gear  # need to do it this way because cmd is gear message type

        # Add ULC messages if needed, not required for now (all functionality works without ULC messages)

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
        """Enable vehicle control using ROS messages"""
        self.is_enabled = True
        msg = Empty()
        self.pub_enable_cmd.publish(msg)

    def disable(self):
        """Disable vehicle control using ROS messages"""
        self.is_enabled = False
        self.zero_dbw_messages()
        # NOTE: Disable is NOT AN EMERGENCY STOP. It simply disables any ROS control over the vehicle.

        # TODO: Verify if this topic actually exists. If not simply return without publishing a message
        # since not publishing the enable message automatically times out control in the dbw hardware after 200ms
        msg = Empty()
        self.pub_disable_cmd.publish(msg)

    # def enable_callback(self, Empty_msg):
    #     """Enable vehicle from callback"""
    #     self.enable()

    # def disable_callback(self, Empty_msg):
    #     """Disable vehicle from callback"""
    #     self.disable()

    def drive_twist_callback(self, Twist_msg):
        """Drive vehicle from twist callback"""

        self.requested_speed = Twist_msg.linear.x
        self.requested_road_angle = Twist_msg.angular.z

        # TODO: Implement throttle, brake, and steering commands

        # TODO: Shift to neutral and then reverse if negative speed

        # TODO: Shift to neutral and then drive if positive speed

        # TODO: Stop vehicle and shift to park if no input for 10 seconds

        pass

    def speed_controller(self, requested_speed):
        pass
        # TODO: Add Controller classes?
        #           - Proportional Speed Controller
        #           - Come to stop and reverse?
        #           - Brake Controller
        #           - Proportional Steering Controller

    # TODO: Test if this works with ROS
    def shift_gear(self, gear_input: str, second_try=False):
        """Shifts gear using string input"""

        gear_input = gear_input.upper()

        gear_dict = {
            "NONE": 0,
            "PARK": 1,
            "REVERSE": 2,
            "NEUTRAL": 3,
            "DRIVE": 4,
            "LOW": 5,
        }  # Only used for internal error checking for this method

        # Error checking
        if gear_input not in gear_dict:
            rospy.logerr(f"Invalid gear: {gear_input}")
            return False

        if self.gear == gear_dict[gear_input]:
            rospy.loginfo(f"Gear is already {gear_input}")
            return True

        if abs(self.speed) > 1:
            if second_try:
                rospy.logerr("Cannot shift gears while vehicle is moving")
                return False

            rospy.logwarn("Cannot shift gear when speed is not near zero. Waiting 2 seconds before retrying...")
            rospy.sleep(2)
            return self.shift_gear(gear_input, second_try=True)

        # Shift gear
        self.msg_gear.gear = gear_dict[gear_input]  # Set gear message
        self.msg_shift_gear.cmd = self.msg_gear  # Make gear shift command
        self.pub_gear.publish(self.msg_shift_gear)
        rospy.loginfo(f"Gear shifted to {gear_input}")
        rospy.sleep(0.5)  # Wait for gear to shift to happen in the hardware

        return True
