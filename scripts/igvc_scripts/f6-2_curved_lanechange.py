#!/usr/bin/env python3


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.waypoints = actor.read_waypoints(file_path="/home/dev/curved_waypoint.yaml")


def case_1():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    actor.drive_for(speed=3, angle=35, speed_distance=3)

    actor.drive_for(speed=3, angle=0, speed_distance=2)

    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=3.15)


def case_2():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )
    actor.drive_for(speed=3, angle=-30, speed_distance=3)
    actor.drive_for(speed=3, angle=40, speed_distance=7)
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )
    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=4.0)


def case_3():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    actor.drive_for(speed=3, angle=20, speed_distance=2.25)

    actor.drive_for(speed=3, angle=-40, speed_distance=7)

    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.9)


def case_4():
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    actor.drive_for(speed=3, angle=-35, speed_distance=4)

    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 3.0},
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.5)


actor.print_title("Q3 - Lane Keeping")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until barrel is detected")

case_2()

actor.print_highlights("Q3 - Lane Keeping Complete!")

# ---------------------------------------------------------------------------------------------------------------------
