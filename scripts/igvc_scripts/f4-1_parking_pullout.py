#!/usr/bin/env python3

# Test FIV.1 Parking. Pull Out
# 1. Test Goal
# This test is intended to evaluate if a vehicle is able to reverse out (or pull out) of the representative parking space.
# The direction of pull out (right-turn-pull-out or left-turn-pull-out) is selected by the judges. The same direction is
# repeated for all 3 attempts.
# Figure 15: Parking. Pull Out
# 2. Test Setup
# The following items shall be placed on the road:
# o Barrel 1 to indicate a starting point at which vehicle is stationary
# o Barrel 2 to indicate an ending point
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle slowly pulls out from the parking spot
# 5. Vehicle reaches full stop within 3 ft from the Barrel 2
# 6. End test run

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

estop.enable_dbw()  # Enable vehicle control via ROS - one time message
# Pull Out To Right
# actor.print_title("F4.1 Parking Pull Out Right")

# actor.drive_for(speed=1.5, angle=0.0, speed_distance=0.75)

# actor.drive_for(speed=3.0, angle=-35.0, speed_distance=6.0)

# actor.drive_for(
#     speed=1.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 3.0}
# )

# actor.stop_vehicle(duration=5.0, using_brakes=True)

actor.print_title("F4.1 Parking Pull Out Left")

actor.drive_for(speed=2.0, angle=0.0, speed_distance=2.0)

actor.drive_for(speed=3.0, angle=27.0, speed_distance=6.0)

actor.drive_for(
    speed=1.5, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 3.0}
)

actor.stop_vehicle(duration=5.0, using_brakes=True)

actor.print_highlights("Parking Pull Out Complete!")
