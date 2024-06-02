#!/usr/bin/env python3

# Test Q.6 Right Turn
# 1. Test Goal
# This test is intended to evaluate if the vehicle is able to make a right turn, merge into the lane and drive
# within a lane until an obstacle is detected.
# Figure 8: Qualification Testing. Right Turn
# 2. Test Setup
# The following items shall be placed on the road:
# o Barrel 1 to indicate starting point at which vehicle is stationary. The Barrel 1 could be placed
# near the stop bar, or several feet away from the stop bar per judges’ decision.
# o Barrel 2 to indicate an ending point. The barrel is placed about 30 ft away from the stop bar
# in the right lane
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle takes off from full stop at Barrel 1
# 4. Vehicle maintains the target speed (between 3 – 5 mph)
# 5. Vehicle makes right turn and merges into correct lane
# 6. Vehicle maintains the target speed (between 3 – 5 mph)
# 7. Vehicle reaches full stop within 5 ft from the Barrel 2
# 8. End test run
# 4. Evaluation
# Pass Criteria - vehicle is able to turn right, merge into correct lane and stop without hitting abarrel
# or crossing boundaries


import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("Q6 - Right Turn")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until barrel is detected")

actor.print_highlights("Intersection")

# actor.drive_for(speed=3.0, angle=actor.lane_center, end_function=actor.yolo_look_for, stop_sign=True, size=100)

actor.print_highlights("Turning Right")

actor.drive_for(speed=4.0, angle=0.0, speed_distance=0.5)

actor.drive_for(speed=4.0, angle=-30.0, speed_distance=6.5)

# Pass functions to drive_for() to drive with function based steering until a custom end condition is met.
actor.drive_for(
    speed=4.0, angle=actor.lane_center, end_function=actor.lidar_3d, end_function_kwargs={"max_distance": 5.0}
)

# actor.drive_for(speed-4.0, angle=actor.lane_center, speed_distance=10.0)

actor.stop_vehicle(duration=15.0, using_brakes=True, softness=0.1, brake_distance=1.75)

actor.print_highlights("Q3 - Lane Keeping Complete!")


# Ex: if needed disable or estop can be triggered anywhere

# estop.disable_dbw()  # Disable vehicle control via ROS - one time message
# # NOTE: ^ This is not an E-Stop. It just disables vehicle control
# # OR
# estop.trigger_e_stop()
# # OR
# estop()  # same as above

# ---------------------------------------------------------------------------------------------------------------------
