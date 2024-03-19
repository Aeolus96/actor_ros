#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here

actor.print_title("Testing - Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.drive_for(speed=1.0, angle=0.0, duration=5.0)

actor.stop_vehicle(duration=5.0)

actor.print_highlights("Testing - Go Forward Complete!")