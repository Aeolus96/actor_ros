#!/usr/bin/env python3

# Test Q.5 Left Turn
# 1. Test Goal
# This test is intended to evaluate if a vehicle is able to make a left turn across the traffic, merge into
# expected lane and drive within this lane until an obstacle is detected.
# 2. Test setup
# The following items shall be placed on the road:
# o Barrel 1 to indicate a starting point at which vehicle is stationary. The Barrel 1 could be placed
# near the stop bar, or several feet away from the stop bar per judges’ decision.
# o Barrel 2 to indicate an ending point. The barrel is placed about 30 ft away from the stop bar
# in the right lane
# 3.Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle maintains the target speed (between 3 – 5 mph)
# 5. Vehicle turns left across the traffic and merges into correct lane
# 6. Vehicle maintains the target speed (between 3 – 5 mph)
# 7. Vehicle reaches full stop within 5 ft from the Barrel 2
# 8. End test run
# 4. Evaluation
# Pass Criteria - vehicle is able to turn left, merge into correct lane and stop without hitting abarrel
# or crossing boundaries

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------------------------------------------------------------------------
actor.print_title("Q5 - Left Turn")

# estop.reset()  # Reset E-Stop if needed - Preferably this should done manually via the GUI
estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Intersection")

# Pass functions to drive_for() to drive with function based steering until a custom end condition is met.
# actor.drive_for(speed=4.0, angle=actor.lane_center, speed_distance=2.0)

actor.print_highlights("Turning Left")

actor.drive_for(speed=3.0, angle=0.0, speed_distance=3.5)

actor.drive_for(speed=3.0, angle=25.0, speed_distance=7.5)

actor.print_highlights("Stopping at Barrel")

actor.drive_for(
    speed=4.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 5.0}
)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=3.0)

actor.print_highlights("Q5 - Left Turn Complete!")

# ---------------------------------------------------------------------------------------------------------------------
