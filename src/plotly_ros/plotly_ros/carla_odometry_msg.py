#!/usr/bin/env python3
"""
Subscribe to /carla/hero/odometry and print x-y positions (plus yaw) in real time.
Works on ROS 2 Foxy, Humble, Iron, etc.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import atan2

def quaternion_to_yaw(q):
    """
    Convert geometry_msgs/Quaternion to yaw (rotation about Z), in radians.
    """
    # ROS uses (x, y, z, w) order
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

class OdomPrinter(Node):
    """
    Simple subscriber node that logs each 2-D pose estimate.
    """

    def __init__(self):
        super().__init__('odom_printer')
        self.create_subscription(
            Odometry,
            '/carla/hero/odometry',
            self.odom_callback,
            10,            # queue size
        )
        self.get_logger().info('Listening to /carla/hero/odometry …')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)

        # Format to two decimals so the log stays compact
        self.get_logger().info(f'Position (x, y) = ({p.x:.2f}, {p.y:.2f})   yaw = {yaw:.2f} rad')

def main():
    rclpy.init()
    node = OdomPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

