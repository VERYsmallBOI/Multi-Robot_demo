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
        super().__init__('robot1_follow_robot2')

        self.robot2_pose = None
        self.robot1_pose = None
        self.robot2_cmd = TwistStamped()

        # Subscriptions
        self.create_subscription(Odometry, '/robot3/odom', self.robot2_odom_callback, 10)
        self.create_subscription(Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.create_subscription(TwistStamped, '/robot3/cmd_vel', self.robot2_cmd_callback, 10)

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

    def robot2_cmd_callback(self, msg):
        self.robot2_cmd = msg

    def control_loop(self):
        if not self.robot1_pose or not self.robot2_pose:
            return

        x1, y1, yaw1 = self.robot1_pose
        x2, y2, yaw2 = self.robot2_pose

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx ** 2 + dy ** 2)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if distance > 0.5:
            # Move closer to robot2 with heading correction
            target_angle = math.atan2(dy, dx)
            angle_diff = normalize_angle(target_angle - yaw1)

            msg.twist.linear.x = max(0.2, 0.15 * distance)
            msg.twist.angular.z = 0.5 * angle_diff

        else:
            # Within 0.5m: match orientation with robot2
            angle_diff = normalize_angle(yaw2 - yaw1)

            if abs(angle_diff) > 0.05:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.6 * angle_diff
            else:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
