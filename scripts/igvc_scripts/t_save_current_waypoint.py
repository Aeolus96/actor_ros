#!/usr/bin/env python3

# NOTE: This script is not part of an IGVC test.
# Saves the current waypoint to a specified YAML file.

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
import os
import time

from actor_ros.actor_tools import YAMLReader

yaml_path = "/home/dev/waypoint_test.yaml"


yml = YAMLReader(file_path=yaml_path)

start_time = rospy.Time.now()
while rospy.Time.now() - start_time < rospy.Duration(60):
    actor.update_current_waypoint()
    num_waypoints = len(yml.params)
    # yml.params.update({f"waypoint{num_waypoints}" : {"lat" : actor.waypoint.latitude}, {"lon" : actor.waypoint.longitude}, {"heading" : actor.waypoint.heading}})
    yml.params[f"waypoint{num_waypoints}"] = (
        {"lat": actor.waypoint.lat},
        {"lon": actor.waypoint.long},
        {"heading": actor.waypoint.current_heading},
    )
    # yml.params.update({f"waypoint{num_waypoints}" : actor.waypoint})
    # yml.params[f'waypoint{num_waypoints}'] = actor.waypoint
    print(actor.waypoint)
    time.sleep(0.2)

yml.write()
actor.print_highlights(f"Saved waypoint to {yaml_path}.")
yml.read()
actor.print_highlights(f"Current waypoints: {yml.params}")
