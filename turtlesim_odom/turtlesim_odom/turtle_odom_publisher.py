#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


class TurtleOdomPublisher(Node):
    def __init__(self):
        super().__init__('turtle_odom_publisher')
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/turtle1/odom', 10)
        self.get_logger().info("Turtle Odom Publisher started")

    def pose_callback(self, msg: Pose):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0

        # Orientation (convert theta -> quaternion)
        q = Quaternion()
        q.z = math.sin(msg.theta / 2.0)
        q.w = math.cos(msg.theta / 2.0)
        odom.pose.pose.orientation = q

        # Velocities
        odom.twist.twist.linear.x = msg.linear_velocity
        odom.twist.twist.angular.z = msg.angular_velocity

        self.odom_pub.publish(odom)
        self.get_logger().info(
            f"Published Odom: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtleOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
