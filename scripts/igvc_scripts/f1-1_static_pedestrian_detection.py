#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

actor.print_title("F1.1 Static Pedestrian Detection")

actor.print_highlights("Detecting Pedestrians")

count = 0

while count < 300:
    actor.yolo_look_for(person=True, size=100)
    count += 1
    rospy.sleep(0.1)

actor.print_highlights("Static Pedestrian Detection Complete!")
