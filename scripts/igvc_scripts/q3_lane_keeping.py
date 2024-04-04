#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------


# Custom end condition as a function just for this script. Functions like these can added to module as needed...
def stop_at_barrel() -> bool:
    distance = actor.msg_lidar_zone_0_closest.data
    return 0 < distance < 3.0


# ---------------------------------------------------------------------------------------------------------------------
actor.print_title("Q3 - Lane Keeping")

# estop.reset()  # Reset E-Stop if needed - Preferably this should done manually via the GUI
estop.enable_dbw()  # Enable vehicle control via ROS - one time message

actor.print_highlights("Lane keeping until barrel is detected")

# Pass functions to drive_for() to drive with function based steering until a custom end condition is met.
actor.drive_for(speed=3.0, angle=actor.lane_center, function=stop_at_barrel)

actor.stop_vehicle(duration=5.0)

actor.print_highlights("Q3 - Lane Keeping Complete!")


# Ex: if needed disable or estop can be triggered anywhere

# estop.disable_dbw()  # Disable vehicle control via ROS - one time message
# # NOTE: ^ This is not an E-Stop. It just disables vehicle control
# # OR
# estop.trigger_estop()
# # OR
# estop()  # same as above

# ---------------------------------------------------------------------------------------------------------------------
