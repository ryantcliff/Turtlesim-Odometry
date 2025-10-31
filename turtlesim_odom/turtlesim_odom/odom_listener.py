#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry, '/turtle1/odom', self.odom_callback, 10)
        self.get_logger().info("Odom Listener started")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        self.get_logger().info(
            f"Odom → x: {pos.x:.2f}, y: {pos.y:.2f}, "
            f"linear_vel: {lin.x:.2f}, angular_vel: {ang.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
