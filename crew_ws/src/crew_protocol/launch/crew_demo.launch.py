#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    # CREW Coordinator
    coordinator = Node(
        package='crew_protocol',
        executable='coordinator',
        name='crew_coordinator',
        output='screen'
    )
    
    # CREW Multi-robot system
    robots = Node(
        package='crew_protocol',
        executable='multi_robot',
        name='crew_multi_robot',
        output='screen'
    )
    
    # CREW Broadcast (delayed to let robots start)
    broadcast = TimerAction(
        period=3.0,
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
        coordinator,
        robots,
        broadcast,
    ])
