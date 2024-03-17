#!/usr/bin/env python3

import actor_ros  # ACTor specific utility functions
import rospy  # ROS Python API

estop = actor_ros.actor_tools.EStopManager()  # E-Stop Manager instance

actor = actor_ros.scripting_tools.ActorScriptTools()  # ACTor Scripting Tools instance
# ^ This starts everything that needs to be up and running for the script

# TODO: Add your code from here

import sys
import time

print("Hello World!", file=sys.stdout)
print("Error!", file=sys.stderr)
for i in range(20):
    print(i)
    time.sleep(1)
print("Error!", file=sys.stderr)
print("Goodbye World!")
