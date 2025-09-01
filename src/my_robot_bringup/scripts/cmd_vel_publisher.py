#!/usr/bin/env python3  # ADD THIS LINE AT THE VERY TOP
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)  # Publish every 0.5 seconds
        
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0  # No rotation
        try:
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: linear.x={msg.linear.x} angular.z={msg.angular.z}')
        except:
            self.get_logger().warn('Failed to publish - ROS may be shutting down')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('CmdVelPublisher stopped by user')
    finally:
        # Only try to shutdown if ROS is still valid
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # ROS already shutting down

if __name__ == '__main__':
    main()