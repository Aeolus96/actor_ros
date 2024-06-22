#!/usr/bin/env python3

import time

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API
from actor_ros.actor_tools import YAMLReader

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance

# Load waypoints from YAML file
file_path = "/home/dev/waypoint_test.yaml"
yaml_file = YAMLReader(file_path=file_path)


# Drive manually to reach waypoints
start_time = rospy.Time.now()
while rospy.Time.now() - start_time < rospy.Duration(10):
    actor.update_current_waypoint()  # Update current waypoint to current ACTor position
    num_waypoints = len(yaml_file.params)
    yaml_file.params[f"waypoint{num_waypoints}"] = (
        {"lat": actor.waypoint.lat},
        {"long": actor.waypoint.long},
        {"heading": actor.waypoint.heading},
    )
    print(actor.waypoint)
    time.sleep(0.2)

# Write waypoints to YAML file
yaml_file.write()
actor.print_highlights(f"Saved waypoint to {file_path}.")
yaml_file.read()
actor.print_highlights(f"Current waypoints: {yaml_file.params}")
