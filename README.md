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
## Run

To run, clone this repository into the src folder within your ROS2 workspace.

Note: If you have not created a workspace previously, you will need to create one: [Creating a ROS2 Workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

To source your ROS2 installation, use:
```
source /opt/ros/jazzy/setup.bash
```

Open a terminal in the src folder in your ROS2 workspace, and source your ROS installation. Then run:

```
colcon build
```
```
source install/local_setup.bash
```
```
ros2 launch drone_delivery robot_launch.py
```

The simulation will then ask if you would like to simulate one or two drones. After this, the program will open Webots and the simulation will begin.

## Credits

Drone mathematics based upon example from the [Webots User Guide](https://cyberbotics.com/doc/guide/mavic-2-pro?version=R2022b) used under [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
