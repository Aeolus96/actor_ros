#!/usr/bin/env python3

# Test Q.1 E-Stop Manual
# 1. Test Goal
# This test is intended to evaluate safety features of Manual E-Stop.
# 2. Test Setup
# The following items shall be placed on the road:
# o Barrel 1 on the side of the road to indicate a starting point at which vehicle is stationary
# o Barrel 2 on the side of the road to indicate the position where E-Stop button is pressed
# o Barrel 3 on the side of the road to indicate the maxim distance for the vehicle to come to the complete
# stop. The distance between Barrel 2 and Barrel 3 is 14 feet
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle maintains the target speed
# 5. Judge manually pushes E-Stop at Barrel 2
# 6. Vehicle comes to full stop before reaching Barrel 3.
# 7. End test run
# 4. Evaluation
# Pass Criteria - vehicle is able to stop before reaching Barrel 3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

# TODO: Add your code from here

actor.print_title("F5.3 Static Pedestrian Lane Change")

actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.drive_for(
    speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 10.0}
)

actor.drive_for(speed=3, angle=25, speed_distance=2)

actor.drive_for(speed=3, angle=0, speed_distance=3.5)

actor.drive_for(speed=3, angle=-25, speed_distance=2)

actor.drive_for(
    speed=3, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 2.5}
)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=2.95)

actor.print_highlights("Pedestrian Lane Change Complete!")
