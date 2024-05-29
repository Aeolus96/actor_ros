#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("Testing - Lidar")

while True:
    actor.lidar_3d(lidar_zone="front", verbose=True)
    rospy.sleep(0.5)

# actor.print_highlights("Testing - Following Waypoints Complete!")
