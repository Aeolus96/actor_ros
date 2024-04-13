# Actor ROS Package

This package enables abstraction of the LTU ACTor autonomous development vehicle in ROS.

NOTE: UNDER ACTIVE CONSTRUCTION...

> Use the newer sim package!!
> https://github.com/LTU-Actor/igvc_python_simulator.git

Commands:

```bash
# Run the install script to install dependencies. It also runs the requirements.txt file automatically
./install.sh

# Build the package
catkin build actor_ros

# Source ROS workspace

# Launch
roslaunch actor_ros actor.launch simulated:=true
```
