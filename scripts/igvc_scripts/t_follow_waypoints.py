#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("Testing - Following Waypoints")

# Load waypoints from YAML file
actor.waypoints = actor.read_waypoints(file_path="/home/dev/waypoint_test.yaml")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

# Follow waypoints until last waypoint is reached
actor.drive_for(
    speed=5.0,
    angle=actor.follow_waypoints,
    angle_kwargs={"radius": 1.5, "gain": 1.0},
    end_function=actor.waypoint_in_range,
    end_function_kwargs={"goal_waypoint": actor.waypoints[-1], "radius": 3.0},
)
# actor.drive_for(speed=1.5, angle=0.0, duration=8.0) # debug line to check dbw

actor.stop_vehicle(duration=5.0, using_brakes=True)

actor.print_highlights("Testing - Following Waypoints Complete!")
