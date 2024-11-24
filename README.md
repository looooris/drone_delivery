# A simple drone delivery project in Webots and ROS2. 

**A CS4048 asssessment. Group 12.**

## Prerequisites

Ubuntu-based system

[Installing ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

[Installing Webots](https://cyberbotics.com/doc/guide/installation-procedure)

Webots / ROS2 Interface
```
sudo apt-get install ros-jazzy-webots-ros2
```
Colcon 
```
sudo apt install colcon
```
There are no python dependencies required for the project except those above. The drone_control_services package provides custom messages and servcies for use within the main drone_delivery package.

## Run

To run, clone this repository into the src folder within your ROS2 workspace.

Note: If you have not created a workspace previously, you will need to create one: [Creating a ROS2 Workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)


Open a terminal in the src folder in your ROS2 workspace, and source your ROS installation.
To source your ROS2 installation, use:
```
source /opt/ros/jazzy/setup.bash
```
Then run:
```
colcon build
```
```
source install/local_setup.bash
```
```
ros2 launch drone_delivery robot_launch.py
```

The terminal will then ask if you would like to simulate one or two drones. After this, the program will open Webots and the simulation will begin.

Note: IF YOU ARE EDITING THE FILES! The program may crash after rebuilding. To fix this, close your terminal, clear out the colcon install waste (the `build`, `install` & `log` folders from your ROS2 workspace), and re-follow the steps above. I'm not sure why this happens, but I think that editing the things that depend on the custom actions can break the program. Sometimes, this causes errors when re-launching the program as well, especially if you are switching between one and two drone operations.

## Credits

Drone mathematics based upon example from [patrickpbarroso](https://github.com/patrickpbarroso/drone-simulation) used under [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
