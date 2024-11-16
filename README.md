# A simple drone delivery project in Webots and ROS2. 

**A CS4048 asssessment. Group 12.**

## Prerequisites

Ubuntu-based system

[Installing ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

[Installing Webots](https://cyberbotics.com/doc/guide/installation-procedure)

Installing the Webots / ROS2 Interface:
```
sudo apt-get install ros-jazzy-webots-ros2
```
## Run

To run, clone this repository into a folder within your ROS2 workspace (usually /home/ros2-ws/ on Ubuntu).

Open a terminal at the root of your clone of this repository, and source your ROS installation. Then run:

```
colcon build
```
```
source install/local_setup.bash
```
```
ros2 launch drone_delivery robot_launch.py
```

The simulation should then run as intended.

## Credits

Drone mathematics based upon example from the [Webots User Guide](https://cyberbotics.com/doc/guide/mavic-2-pro?version=R2022b) used under [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
