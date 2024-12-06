# A simple drone delivery project in Webots and ROS2. 

**A CS4048 asssessment. Group 12.**

## Prerequisites

Ubuntu-based system

ROS2: [Installing ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Webots: [Installing Webots](https://cyberbotics.com/doc/guide/installation-procedure)

Webots / ROS2 Interface
```
sudo apt-get install ros-jazzy-webots-ros2
```
Colcon 
```
sudo apt install colcon
```
There are no python dependencies required for the project aside from what is included in ROS, Webots and the interface above. The drone_control_services package provides custom messages and servcies for use within the main drone_delivery package but will be detected by ROS2 upon running `colcon build` and sourcing the `local_setup.bash` file.

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

At the end of the simulation, the program will write the tasks that the drone completed and time taken to complete these in a file called `locations.csv` inside the `drone_delivery` folder.

Note: IF YOU ARE EDITING THE FILES! The program may crash after rebuilding. To fix this, close your terminal, clear out the colcon install waste (the `build`, `install` & `log` folders from your ROS2 workspace), and re-follow the steps above. I'm not sure why this happens, but I think that editing the things that depend on the custom actions can break the program. Sometimes, this causes errors when re-launching the program as well, especially if you are switching between one and two drone operations.

Update: 26/11 - this seems to have fixed itself. I'm not sure why but I'm keeping this here incase it breaks itself again.

## Credits

Drone mathematics based upon example from The [Webots Guide](https://github.com/cyberbotics/webots/blob/master/projects/robots/dji/mavic/controllers/mavic2pro_patrol/mavic2pro_patrol.py) used under [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
