import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('drone_delivery')
    robot_description_path = os.path.join(package_dir, 'resource', 'robots.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'droneSimulationWorld.wbt'),
        #ros2_supervisor= True
    )

    #ros2_supervisor = Ros2SupervisorLauncher()

    drone_control = WebotsController(
            robot_name="drone_one",
            parameters=[
                {'robot_description': robot_description_path},
            ],
        )

    schedule  = Node(
        package="drone_delivery",
        executable="schedule",
    )

    return LaunchDescription([
        webots,
        schedule,
        #webots._supervisor,
        #ros2_supervisor,
        drone_control,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])