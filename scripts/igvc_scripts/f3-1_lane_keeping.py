#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------


# def white_line():
#     return actor.msg_bumper_camera_bumper_line_detected.data


# previous_size = 0


# def slow_to_sign(speed_max: float = 4.0, distance: int = 70, gain: float = 31):
#     # min(brake_target, (1 / max(self.lidar_2d() - brake_distance, 0.1)) / 10)
#     speed = speed_max / max((actor.msg_region_right_closest - distance / gain), 1)
#     return speed


actor.print_title("F3.1 - Lane Keeping")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until stop sign is detected")

# actor.drive_for(
#     speed=3.0,
#     angle=actor.lane_center,
#     end_function=actor.yolo_look_for,
#     end_function_kwargs={"stop_sign": True, "size": 60},
# )

# actor.drive_for(speed=0.5, angle=actor.lane_center, end_function=white_line)

actor.drive_for(
    speed=4,
    angle=actor.lane_center,
    end_function=actor.lidar_3d,
    end_function_kwargs={"lidar_zone": "right", "max_distance": 4.30},
)

actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, sign_distance=1.75)

actor.print_highlights("Lane keeping until barrel is detected")

actor.drive_for(speed=3.0, angle=0.0, speed_distance=10.0)

actor.drive_for(
    speed=3.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.75}
)

actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, brake_distance=2.95)

# actor.print_highlights("F3.1 - Lane Keeping Complete!")

# ---------------------------------------------------------------------------------------------------------------------
