#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crew_interfaces.msg import EmergencyRequest
import time

class CrewBroadcast(Node):
    def __init__(self):
        super().__init__('crew_broadcast')
        
        self.publisher = self.create_publisher(
            EmergencyRequest,
            'crew/emergency_broadcast',
            10
        )
        
        self.get_logger().info('CREW Broadcast Node Started')
        
    def broadcast_emergency(self, emergency_type, lat, lon, radius, capabilities):
        msg = EmergencyRequest()
        msg.emergency_type = emergency_type
        msg.latitude = lat
        msg.longitude = lon
        msg.radius_km = radius
        msg.capabilities_needed = capabilities
        msg.time_window_hours = 4
        msg.auth_token = 'DEMO_TOKEN_12345'  # Simplified for demo
        msg.timestamp = int(time.time())
        
        self.publisher.publish(msg)
        
        self.get_logger().info(f'📡 CREW BROADCAST: {emergency_type}')
        self.get_logger().info(f'   Location: ({lat}, {lon})')
        self.get_logger().info(f'   Needed: {", ".join(capabilities)}')

def main(args=None):
    rclpy.init(args=args)
    node = CrewBroadcast()
    
    node.get_logger().info('Waiting 3 seconds before broadcasting emergency...')
    time.sleep(3)
    
    node.broadcast_emergency(
        emergency_type='WILDFIRE',
        lat=37.7749,
        lon=-122.4194,
        radius=5.0,
        capabilities=['THERMAL_IMAGING', 'DEBRIS_CLEARING', 'ROUTE_MAPPING']
    )
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
