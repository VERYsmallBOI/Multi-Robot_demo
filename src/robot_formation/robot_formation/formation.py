#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class FollowRobot(Node):
    def __init__(self):
        super().__init__('formation')

        self.robot2_pose = None
        self.robot1_pose = None

        # Subscriptions
        self.create_subscription(Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        self.create_subscription(Odometry, '/robot1/odom', self.robot1_odom_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def robot2_odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot2_pose = (pos.x, pos.y, yaw)

    def robot1_odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot1_pose = (pos.x, pos.y, yaw)

    def control_loop(self):
        if not self.robot1_pose or not self.robot2_pose:
            return

        # Extract poses
        x1, y1, yaw1 = self.robot1_pose
        x2, y2, yaw2 = self.robot2_pose

        # Desired offset: 1m at 120° from robot2's heading (60° left behind)
        r = 1.0
        theta_offset = math.radians(120)
        offset_x = r * math.cos(theta_offset)
        offset_y = r * math.sin(theta_offset)

        # Formation target in global frame
        target_x = x2 + offset_x * math.cos(yaw2) - offset_y * math.sin(yaw2)
        target_y = y2 + offset_x * math.sin(yaw2) + offset_y * math.cos(yaw2)

        # Pursue target point
        dx = target_x - x1
        dy = target_y - y1
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = normalize_angle(angle_to_target - yaw1)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if distance > 2.0:
            # Far from formation point → go fast
            msg.twist.linear.x = 0.3
            msg.twist.angular.z = 0.8 * angle_diff
        else:
            # Controlled approach to formation point
            msg.twist.linear.x = min(0.2, 0.15 * distance)
            msg.twist.angular.z = 0.5 * angle_diff

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
