# Actor ROS Package

This package enables abstraction of the LTU ACTor autonomous development vehicle in ROS.

NOTE: UNDER ACTIVE CONSTRUCTION...

Commands:

```bash
roslaunch dbw_polaris_can dbw.launch

# Calibrate steering once at the beginning
rostopic pub -1 /vehicle/calibrate_steering std_msgs/Empty "{}"

# Enable vehicle continuously to use via ROS
rostopic pub /vehicle/enable std_msgs/Empty "{}"

```
