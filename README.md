# Actor ROS Package

This package enables abstraction of the LTU ACTor autonomous development vehicle in ROS; adds a unified status message and control interface for the ACTor vehicle.

## Dependencies

- ROS Noetic
- [`dbw_polaris_msgs`](https://bitbucket.org/DataspeedInc/dbw_polaris_ros/src/master/ROS_SETUP.md): ROS messages for controlling a DataSeed Drive-By-Wire system installed in ACTor.
    > This external package needs to be installed manually
- ACTor computer/laptop setup using [ACTOR setup](https://github.com/LTU-Actor/setup.git)
    > Only need if setting up the ACTor computer/laptop for the first time

## Installation

### Download `actor_ros` package

```bash
# Go to <path_to_your_catkin_ws>/src/ directory
cd <path_to_your_catkin_ws>/src/

# Clone the repository
git clone https://github.com/Aeolus96/actor_ros.git
```

### Install Dependencies

```bash
# Go inside the cloned ROS package.
cd <path_to_your_catkin_ws>/src/actor_ros

# IMPORTANT: Make sure you are inside the actor_ros directory before running the install script

# Set executable permissions and Run the install script
chmod +x ./install.sh
./install.sh
```

> Watch for errors in the output. Warnings are normal for some of the dependencies.

```bash
# If later you encounter any import errors in python, you can manually run the following command from this directory
pip3 install -r requirements.txt
```

## Usage

You can use the [IGVC Simulator](https://github.com/LTU-Actor/igvc_python_simulator.git) with this package. Make sure to remove older versions of the simulator, install the latest version and then launch actor_ros:

```bash
# Source ROS workspace
source <path_to_your_catkin_ws>/devel/setup.bash

# Launch with real vehicle
roslaunch actor_ros actor.launch

# OR Launch with simulated environment
roslaunch actor_ros actor.launch simulated:=true
```

> Please make sure the IP addresses are correct between the two ACTor vehicles in the `support_real.launch` file

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

## License

This software is released under the [MIT License](LICENSE).
