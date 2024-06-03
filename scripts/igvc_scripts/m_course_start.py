#!/usr/bin/env python3


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
estop.enable_dbw()  # Enable vehicle control via ROS - one time message ^ This starts everything that needs to be up and running for the script


global real
real = True


# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
def m1():
    actor.print_title("M1 - Main Course Start")

    # actor.drive_for(speed=3.0, angle=actor.lane_center, end_function=actor.yolo_look_for, stop_sign=True, size=100)
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/main_1_waypoint.yaml")
    actor.drive_for(
        speed=3.0,
        angle=actor.follow_waypoints,
        angle_kwargs={"radius": 1.5, "gain": 1.0},
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )
    # actor.print_highlights("Turning Right")

    # actor.drive_for(speed=3.0, angle=0.0, speed_distance=0.75)

    # actor.drive_for(speed=3.0, angle=-30.0, speed_distance=6.0)

    actor.print_highlights("Obstacle Lane Change")

    # actor.drive_for(
    #     speed=3,
    #     angle=actor.lane_center,
    #     angle_kwargs={"blob_gain": 55},
    #     end_function=actor.lidar_3d,
    #     end_function_kwargs={"max_distance": 6.0},
    # )

    # actor.drive_for(speed=3, angle=25, speed_distance=2)

    actor.drive_for(speed=3, angle=0, speed_distance=3.8)

    actor.drive_for(speed=3, angle=-25, speed_distance=2)

    actor.drive_for(speed=3, angle=actor.lane_center, speed_distance=8)

    actor.drive_for(speed=3, angle=-25, speed_distance=2)
    actor.drive_for(speed=3, angle=15, speed_distance=1.5)


def m2():
    actor.print_highlights("Stop at Stop Sign")

    actor.drive_for(
        speed=3.5,
        angle=actor.lane_center,
        end_function=actor.lidar_3d,
        end_function_kwargs={"lidar_zone": "right", "max_distance": 4.0},
    )
    actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, sign_distance=1.65)


def m3():
    actor.print_highlights("Right Turn")
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/m3_waypoint.yaml")

    # actor.drive_for(speed=3.0, angle=0.0, speed_distance=2)

    # actor.drive_for(speed=3.0, angle=-30.0, speed_distance=6.0)

    actor.print_highlights("Pedestrian Stopping")

    # Pedestrian
    actor.drive_for(
        speed=3,
        angle=actor.follow_waypoints,
        angle_kwargs={"radius": 1.5, "gain": 1.0},
        end_function=actor.lidar_3d,
        end_function_kwargs={"max_distance": 5.0},
    )

    while actor.lidar_3d(max_distance=7):
        print("waiting")
        actor.stop_vehicle(duration=2.0, using_brakes=True, softness=0.1, brake_distance=5)

    actor.print_highlights("Waypoint Crossing")
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/main_2_waypoint.yaml")
    actor.drive_for(
        speed=3.0,
        angle=actor.follow_waypoints,
        angle_kwargs={"radius": 1.5, "gain": 1.0},
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )


def m4():
    actor.print_highlights("Lane Following")
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/main_4_waypoint.yaml")
    actor.drive_for(
        speed=2.5,
        angle=actor.lane_center,
        angle_kwargs={"blob_gain": 60},
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )
    actor.print_highlights("Switching Lanes")

    actor.drive_for(speed=3, angle=25, speed_distance=2)
    actor.drive_for(speed=3, angle=0, speed_distance=2.25)
    actor.drive_for(
        speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 7.0}
    )

    actor.print_highlights("Barrel Detected")
    actor.drive_for(speed=3, angle=-25, speed_distance=2)
    actor.drive_for(speed=3, angle=0, speed_distance=2.25)


def m5():
    actor.print_highlights("Stop at Stop Sign")

    # Pass functions to drive_for() to drive with function based steering until a custom end condition is met.
    actor.drive_for(
        speed=2.5,
        angle=actor.lane_center,
        angle_kwargs={"blob_gain": 60},
        end_function=actor.lidar_3d,
        end_function_kwargs={"lidar_zone": "right", "max_distance": 3.5},
    )
    actor.stop_vehicle(duration=3.0, using_brakes=True, softness=0.1, sign_distance=1.75)


def m6():
    actor.print_title("M6 - Lap 2")

    actor.waypoints = actor.read_waypoints(file_path="/home/dev/main_6_waypoint.yaml")
    actor.drive_for(
        speed=3.0,
        angle=actor.follow_waypoints,
        angle_kwargs={"radius": 1.5, "gain": 1.0},
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )
    # actor.drive_for(
    #     speed=2.5,
    #     angle=actor.lane_center,
    #     angle_kwargs={"blob_gain": 65},
    #     end_function=actor.yolo_look_for,
    #     end_function_kwargs={"tire":True,"size":15}
    # )

    actor.print_highlights("Tire Detected")
    actor.drive_for(speed=3, angle=20, speed_distance=2.5)
    actor.drive_for(speed=3, angle=0, speed_distance=1.5)
    # actor.drive_for(speed=3, angle=-15, speed_distance=2)

    actor.print_highlights("Lane Following")
    actor.drive_for(
        speed=2.5,
        angle=actor.lane_center,
        angle_kwargs={"blob_gain": 60},
        end_function=actor.yolo_look_for,
        end_function_kwargs={"tire": True, "size": 15},
    )
    actor.print_highlights("Tire Detected")
    actor.drive_for(speed=3, angle=-25, speed_distance=2)
    actor.drive_for(speed=3, angle=0, speed_distance=2.25)
    actor.drive_for(speed=3.0, angle=actor.lane_center, speed_distance=4.0)


def m7():
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/is_fake_waypoint.yaml")
    actor.found_stop_sign = False  # Important to reset
    actor.print_highlights("Check for Stop Sign")
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0, "yolo": True},
    )


def m8():
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/merge_waypoint.yaml")
    actor.print_highlights("Turning Left")

    actor.drive_for(speed=3.0, angle=0.0, speed_distance=3.5)

    actor.drive_for(speed=3.0, angle=25.0, speed_distance=7.5)
    actor.drive_for(
        speed=3.0,
        angle=actor.lane_center,
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )

    actor.drive_for(speed=3.0, angle=18, speed_distance=5)
    actor.drive_for(
        speed=2.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.5}
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.95)

    actor.print_highlights("Main Course Complete!")


def m9():
    actor.found_stop_sign = False  # Important to reset
    actor.waypoints = actor.read_waypoints(file_path="/home/dev/merge_fake_waypoint.yaml")
    actor.print_highlights("Turning Left")

    actor.drive_for(
        speed=3.0,
        angle=actor.follow_waypoints,
        angle_kwargs={"radius": 1.0, "gain": 1.0},
        end_function=actor.waypoint_in_range,
        end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
    )
    actor.drive_for(
        speed=2.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.5}
    )

    actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.95)

    actor.print_highlights("Main Course Complete!")


m1()
m2()
m3()
m4()
m5()
m6()
m7()
if actor.found_stop_sign:
    m2()
m9()  # Waypoint Merge

# ---------------------------------------------------------------------------------------------------------------------
