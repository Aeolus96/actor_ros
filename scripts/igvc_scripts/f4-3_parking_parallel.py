#!/usr/bin/env python3

# Test FIV.3. Parking. Parallel
# 1. Test Goal
# This test is intended to evaluate if a vehicle is able to parallel park into the representative parking space. The
# direction of parallel parking (to the right or to the left) is selected by the judges. The same direction is repeated for
# all 3 attempts.
# Figure 17: Functions Testing. Parking. Parallel
# 2. Test Setup
# The following items shall be placed on the road:
# o Barrel 1 to indicate starting point at which vehicle is stationary
# 3. Test Script
# 1. Begin test run
# 2. Judge pushes 'start' button
# 3. Vehicle backs off from full stop at Barrel 1
# 4. Vehicle slowly pulls into the parking spot
# 5. Vehicle reaches full stop. It should be fully in the box without crossing any lines.
# 6. End test run

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

#actor.print_title("F4.3 Parallel Parking")

#actor.print_highlights("Go Forward")

# estop.enable_dbw()  # Enable vehicle control via ROS - one time message

# actor.shift_gear("REVERSE")

#actor.drive_for(speed=-1.5, angle=0.0, speed_distance=0.1)

# actor.drive_for(speed=-1.5, angle=-40.0, speed_distance=1.8)
# actor.drive_for(speed=-1.5, angle=0.0, speed_distance=0.8)

# actor.drive_for(speed=-1.0, angle=40.0, speed_distance=3.0)
# actor.stop_vehicle(duration=4.0, using_brakes='True')

# actor.shift_gear("DRIVE")
# actor.drive_for(speed=1.0, angle=-40.0, speed_distance=1.0)

# actor.drive_for(speed=-1.2, angle=0.0, speed_distance=0.05)

#reverse until rear barrel is 2m away
# actor.drive_for(speed=-1.2, angle=0.0, end_function=actor.lidar3d, lidar_zone='rear', max_distance=2)
#actor.drive_for(speed=-1.2, angle=0.0, end_function=actor.lidar3d,end_function_kwargs={"lidar_zone": 'rear', "max_distance": 2})


# actor.stop_vehicle(duration=0.1)

# actor.stop_vehicle(duration=5.0, using_brakes='True')


#actor.print_highlights("Parallel Parking Right Complete!")
#####################################################################
# Parallel parking Left
#actor.print_highlights("Go Forward")

estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.shift_gear("REVERSE")

#actor.drive_for(speed=-1.5, angle=0.0, speed_distance=0.1)

actor.drive_for(speed=-1.5, angle=40.0, speed_distance=2.4)
actor.drive_for(speed=-1.5, angle=0.0, speed_distance=1.0)

actor.drive_for(speed=-1.0, angle=-40.0, speed_distance=4.0)
actor.stop_vehicle(duration=4.0, using_brakes='True')

actor.shift_gear("DRIVE")
actor.drive_for(speed=1.0, angle=40.0, speed_distance=0.5)

# actor.drive_for(speed=-1.2, angle=0.0, speed_distance=0.05)

#reverse until rear barrel is 2m away
# actor.drive_for(speed=-1.2, angle=0.0, end_function=actor.lidar3d, lidar_zone='rear', max_distance=2)
#actor.drive_for(speed=-1.2, angle=0.0, end_function=actor.lidar3d,end_function_kwargs={"lidar_zone": 'rear', "max_distance": 2})


actor.stop_vehicle(duration=0.1)

actor.stop_vehicle(duration=5.0, using_brakes='True')


actor.print_highlights("Parallel Parking Right Complete!")