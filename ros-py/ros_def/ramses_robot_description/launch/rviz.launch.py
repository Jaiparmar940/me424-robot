"""
rviz.launch.py
==============
Laptop-side launch — starts RViz2 and robot_state_publisher.
Run this on your laptop while robot.launch.py runs on the Pi.

Prerequisites on your laptop
-----------------------------
  1. ROS2 Jazzy installed
  2. ramses_robot_description package built:
       colcon build --packages-select ramses_robot_description
       source install/setup.bash
  3. ROS_DOMAIN_ID matches the Pi (default 0 is fine if on same network)
  4. Both machines on the same network with multicast enabled

Usage
-----
  ros2 launch ramses_robot_description rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    desc_pkg  = get_package_share_directory('ramses_robot_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # robot_state_publisher also runs on the laptop so RViz2 gets TF
    # even if the Pi's RSP hasn't published yet.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
    ])