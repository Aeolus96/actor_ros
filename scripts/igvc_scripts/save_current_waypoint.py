#!/usr/bin/env python3

# NOTE: This script is not part of an IGVC test.
# Saves the current waypoint to a specified YAML file.

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API
actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
from actor_ros.actor_tools import YAMLReader

yaml_path = 'waypoint_test.yaml'

actor.update_current_waypoint()

yml = YAMLReader(file_path=yaml_path)
yml.read()
num_waypoints = len(yml.params)
# yml.params.update({f"waypoint{num_waypoints}" : {"lat" : actor.waypoint.latitude}, {"lon" : actor.waypoint.longitude}, {"heading" : actor.waypoint.heading}})
# yml.params.update({f"waypoint{num_waypoints}" : actor.waypoint})
yml.params[f'waypoint{num_waypoints}'] = actor.waypoint

yml.write()
actor.print_highlights(f"Saved waypoint to {yaml_path}.")
yml.read()
actor.print_highlights(f"Current waypoints: {yml.params}")