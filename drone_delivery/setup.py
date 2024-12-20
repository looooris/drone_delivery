from setuptools import find_packages, setup

package_name = 'drone_delivery'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/droneSimulationWorldOne.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/droneSimulationWorldTwo.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/robots.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Loris for Group 12',
    maintainer_email='l.giuliani.22@abdn.ac.uk',
    description='A simple drone delivery implementation in ROS2 and Webots.',
    license='Apache 2.0',
    tests_require=[''],
    entry_points={
        'console_scripts': [
            'drone_control = drone_delivery.drone_control:main',
            'schedule_one = drone_delivery.location_publisher_one:main',
            'schedule_two = drone_delivery.location_publisher_two:main',
            'grip_one = drone_delivery.gripper_control_one:main',
            'grip_two = drone_delivery.gripper_control_two:main',
            'goal = drone_delivery.goal_handler:main',
        ],
    },
)