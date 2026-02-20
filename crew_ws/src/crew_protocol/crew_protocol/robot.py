#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crew_interfaces.msg import EmergencyRequest, RobotResponse
import math

class CrewRobot(Node):
    def __init__(self, robot_id='drone_01', robot_type='DRONE', 
                 capabilities=None, position=(37.78, -122.42)):
        super().__init__(f'crew_robot_{robot_id}')
        
        if capabilities is None:
            capabilities = ['THERMAL_IMAGING']
            
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.capabilities = capabilities
        self.latitude = position[0]
        self.longitude = position[1]
        self.battery = 100.0
        self.is_busy = False
        self.owner_allows_crew = True
        
        self.emergency_sub = self.create_subscription(
            EmergencyRequest,
            'crew/emergency_broadcast',
            self.handle_emergency,
            10
        )
        
        self.response_pub = self.create_publisher(
            RobotResponse,
            'crew/robot_responses',
            10
        )
        
        self.get_logger().info(f'🤖 Robot {robot_id} online')
        self.get_logger().info(f'   Type: {robot_type}')
        self.get_logger().info(f'   Capabilities: {", ".join(capabilities)}')
        
    def handle_emergency(self, msg):
        self.get_logger().info(f'📩 Received emergency: {msg.emergency_type}')
        
        can_help = any(cap in msg.capabilities_needed for cap in self.capabilities)
        is_safe = self.battery >= 20.0 and not self.is_busy
        in_range = self.calculate_distance(
            self.latitude, self.longitude,
            msg.latitude, msg.longitude
        ) <= msg.radius_km
        
        if can_help and is_safe and self.owner_allows_crew and in_range:
            self.send_response(msg, 'AVAILABLE')
        else:
            reasons = []
            if not can_help: reasons.append('no matching capabilities')
            if not is_safe: reasons.append('safety constraints')
            if not in_range: reasons.append('outside geofence')
            self.get_logger().info(f'❌ Cannot assist: {", ".join(reasons)}')
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat/2) ** 2 + 
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
             math.sin(dlon/2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def send_response(self, emergency_msg, status):
        response = RobotResponse()
        response.robot_id = self.robot_id
        response.robot_type = self.robot_type
        response.capabilities = self.capabilities
        response.status = status
        response.latitude = self.latitude
        response.longitude = self.longitude
        response.battery_percent = self.battery
        
        distance = self.calculate_distance(
            self.latitude, self.longitude,
            emergency_msg.latitude, emergency_msg.longitude
        )
        response.eta_minutes = int(distance * 10)
        
        self.response_pub.publish(response)
        self.get_logger().info(f'✅ Responding: {status} - ETA: {response.eta_minutes} min')

def main(args=None):
    rclpy.init(args=args)
    robot = CrewRobot(
        robot_id='thermal_drone_01',
        robot_type='DRONE',
        capabilities=['THERMAL_IMAGING', 'ROUTE_MAPPING'],
        position=(37.78, -122.42)
    )
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
