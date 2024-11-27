import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('drone_delivery')
    robot_description_path = os.path.join(package_dir, 'resource', 'robots.urdf')

    num_drones = 0
    while num_drones not in [1, 2]:
        num_drones = input("===\nWould you like to simulate one or two drones?\nPlease enter either (1) or (2).\n===\n")
        if not num_drones.isdigit():
            num_drones = 0
        else:
            num_drones = int(num_drones)

    if num_drones == 1:
        # Webots Simulation Load
        webots = WebotsLauncher(
            world=os.path.join(package_dir, 'worlds', 'droneSimulationWorldOne.wbt'),
        )
        
        # Drone Control Node
        drone_control = WebotsController(
                robot_name="drone_one",
                parameters=[
                    {'robot_description': robot_description_path},
                ],
            )

        # Location Publisher Node
        schedule  = Node(
            package="drone_delivery",
            executable="schedule_one",
        )

        # Gripper Control Node
        grip = Node(
            package="drone_delivery",
            executable="grip_one",
        )

        # Goal Publisher Node
        goal  = Node(
            package="drone_delivery",
            executable="goal",
        )

        # Launch Nodes
        return LaunchDescription([
            webots,
            schedule,
            grip,
            drone_control,
            goal,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            )
        ])

    elif num_drones == 2:
        # Webots Simulation Load
        webots = WebotsLauncher(
            world=os.path.join(package_dir, 'worlds', 'droneSimulationWorldTwo.wbt'),
            #ros2_supervisor= True
        )
        drone_one_control = WebotsController(
                robot_name="drone_one",
                parameters=[
                    {'robot_description': robot_description_path},
                ],
            )
        drone_two_control = WebotsController(
                robot_name="drone_two",
                parameters=[
                    {'robot_description': robot_description_path},
                ],
            )

        # Location Publisher Node
        schedule_two = Node(
            package="drone_delivery",
            executable="schedule_two",
        )

        # Gripper Control Node
        grip_one = Node(
            package="drone_delivery",
            executable="grip_one",
        )

        grip_two = Node(
            package="drone_delivery",
            executable="grip_two",
        )

        # Goal Publisher Node
        goal = Node(
            package="drone_delivery",
            executable="goal",
        )

        # Launch Nodes
        return LaunchDescription([
            webots,
            schedule_two,
            grip_one,
            grip_two,
            drone_one_control,
            drone_two_control,
            goal,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            )
        ])
    
    else:
        raise Exception("Invalid input which the error detection didn't detect.")

