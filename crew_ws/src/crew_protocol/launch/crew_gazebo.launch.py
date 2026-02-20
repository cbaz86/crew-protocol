#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    crew_protocol_dir = get_package_share_directory('crew_protocol')
    world_file = os.path.join(crew_protocol_dir, 'worlds', 'crew_emergency.world')
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn Robot 1 - Thermal Drone (at position -3, -3)
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'thermal_drone_01',
            '-database', 'turtlebot3_waffle',
            '-x', '-3.0',
            '-y', '-3.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Spawn Robot 2 - Ground Bot (at position -3, 3)
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ground_bot_01',
            '-database', 'turtlebot3_waffle',
            '-x', '-3.0',
            '-y', '3.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Spawn Robot 3 - Survey Drone (at position 0, -5)
    spawn_robot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'survey_drone_01',
            '-database', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '-5.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # CREW Coordinator
    crew_coordinator = Node(
        package='crew_protocol',
        executable='coordinator',
        name='crew_coordinator',
        output='screen'
    )
    
    # CREW Multi-robot system
    crew_robots = Node(
        package='crew_protocol',
        executable='multi_robot',
        name='crew_multi_robot',
        output='screen'
    )
    
    # CREW Broadcast (delayed to let robots initialize)
    crew_broadcast = TimerAction(
        period=5.0,  # Wait 5 seconds
        actions=[
            Node(
                package='crew_protocol',
                executable='broadcast',
                name='crew_broadcast',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot1,
        spawn_robot2,
        spawn_robot3,
        crew_coordinator,
        crew_robots,
        crew_broadcast,
    ])
