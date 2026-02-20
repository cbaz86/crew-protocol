#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crew_protocol.robot import CrewRobot

def main(args=None):
    rclpy.init(args=args)
    
    # Create multiple robots with different capabilities
    robots = []
    
    # Robot 1: Thermal drone
    robots.append(CrewRobot(
        robot_id='thermal_drone_01',
        robot_type='DRONE',
        capabilities=['THERMAL_IMAGING', 'ROUTE_MAPPING'],
        position=(37.78, -122.42)
    ))
    
    # Robot 2: Ground robot for debris
    robots.append(CrewRobot(
        robot_id='ground_bot_01',
        robot_type='GROUND',
        capabilities=['DEBRIS_CLEARING', 'HEAVY_LIFT'],
        position=(37.77, -122.43)
    ))
    
    # Robot 3: Survey drone
    robots.append(CrewRobot(
        robot_id='survey_drone_01',
        robot_type='DRONE',
        capabilities=['ROUTE_MAPPING', 'AERIAL_PHOTOGRAPHY'],
        position=(37.79, -122.41)
    ))
    
    # Create multi-threaded executor to run all robots
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    for robot in robots:
        executor.add_node(robot)
    
    try:
        executor.spin()
    finally:
        for robot in robots:
            robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
