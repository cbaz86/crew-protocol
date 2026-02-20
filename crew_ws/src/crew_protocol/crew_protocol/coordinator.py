#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crew_interfaces.msg import RobotResponse

class CrewCoordinator(Node):
    def __init__(self):
        super().__init__('crew_coordinator')
        self.available_robots = []
        
        self.subscription = self.create_subscription(
            RobotResponse,
            'crew/robot_responses',
            self.handle_robot_response,
            10
        )
        
        self.get_logger().info('👨‍🚒 CREW Coordinator Dashboard Active')
        self.get_logger().info('Waiting for robot responses...\n')
        
    def handle_robot_response(self, msg):
        robot_info = {
            'id': msg.robot_id,
            'type': msg.robot_type,
            'capabilities': msg.capabilities,
            'status': msg.status,
            'eta': msg.eta_minutes,
            'battery': msg.battery_percent
        }
        
        self.available_robots.append(robot_info)
        self.display_crew_roster()
        
    def display_crew_roster(self):
        print('\n' + '='*60)
        print('CREW AVAILABLE FOR DEPLOYMENT')
        print('='*60)
        
        for i, robot in enumerate(self.available_robots, 1):
            print(f'\n{i}. {robot["id"]} ({robot["type"]})')
            print(f'   Capabilities: {", ".join(robot["capabilities"])}')
            print(f'   Status: {robot["status"]}')
            print(f'   ETA: {robot["eta"]} minutes')
            print(f'   Battery: {robot["battery"]:.1f}%')
        
        print('\n' + '='*60)
        print(f'Total CREW members available: {len(self.available_robots)}')
        print('='*60 + '\n')

def main(args=None):
    rclpy.init(args=args)
    coordinator = CrewCoordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
