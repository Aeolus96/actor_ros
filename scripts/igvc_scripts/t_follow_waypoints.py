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

actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.waypoints = actor.read_waypoints(filepath="/home/dev/waypoint_test.yaml")
actor.drive_for(
    speed=5.0,
    angle=actor.follow_waypoints,
    function = actor.waypoint_in_range,
    lat = actor.waypoints[-1].lat, long = actor.waypoints[-1].long, radius = 1.0
)
# actor.drive_for(speed=1.5, angle=0.0, duration=8.0)

actor.stop_vehicle(duration=5.0)

actor.print_highlights("Testing - Following Waypoints Complete!")
