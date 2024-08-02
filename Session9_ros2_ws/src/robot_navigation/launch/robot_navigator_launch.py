#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='wall_finder_server',
            name='wall_finder_server',
        ),
        Node(
            package='robot_navigation',
            executable='wall_follower',
            name='wall_follower',
        ),
        Node(
            package='robot_navigation',
            executable='lap_time_server',
            name='lap_time_server',
        ),
        Node(
            package='robot_navigation',
            executable='lap_time_client',
            name='lap_time_client',
        ),
        
    ])
