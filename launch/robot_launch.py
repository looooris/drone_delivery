import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def get_ros2_control_spawners(*args):
    # Declare here all nodes that must be restarted at simulation reset
    ros_control_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffdrive_controller']
    )
    return [
        ros_control_node
    ]

def generate_launch_description():
    package_dir = get_package_share_directory('drone_delivery')
    robot_description_path = os.path.join(package_dir, 'resource', 'robots.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'exampleWorld.wbt')
    )

    drone_control = WebotsController(
            robot_name="drone_one",
            parameters=[
                {'robot_description': robot_description_path},
            ],
            respawn=True
        )


    obstacle_avoider = Node(
        package='drone_delivery',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        drone_control,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])