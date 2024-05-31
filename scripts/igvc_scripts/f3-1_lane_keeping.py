#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------


def white_line():
    return actor.msg_bumper_camera_bumper_line_detected.data


actor.print_title("F3.1 - Lane Keeping")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until stop sign is detected")

actor.drive_for(
    speed=3.0,
    angle=actor.lane_center,
    end_function=actor.yolo_look_for,
    end_function_kwargs={"stop_sign": True, "size": 60},
)

actor.drive_for(speed=1.0, angle=actor.lane_center, end_function=white_line)

actor.stop_vehicle(duration=5.0, using_brakes=True, softness=0.5)

actor.print_highlights("Lane keeping until barrel is detected")

actor.drive_for(speed=3.0, angle=0.0, speed_distance=8.0)

actor.drive_for(
    speed=3.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 3.5}
)

actor.stop_vehicle(duration=5.0, using_brakes=True)

actor.print_highlights("F3.1 - Lane Keeping Complete!")

# ---------------------------------------------------------------------------------------------------------------------
