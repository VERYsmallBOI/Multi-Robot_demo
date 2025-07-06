#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class ZigZagDriver(Node):
    def __init__(self):
        super().__init__('zigzag_driver')
        self.publisher = self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_motion)

        self.start_time = time.time()
        self.state = "forward"
        self.last_switch_time = time.time()
        self.turn_direction = 1  # 1 = left, -1 = right

    def update_motion(self):
        now = time.time()
        elapsed = now - self.last_switch_time

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.state == "forward":
            msg.twist.linear.x = 0.05
            msg.twist.angular.z = 0.0

            if elapsed > 5.0:  # Adjust to control how far it goes (5s ~ 1.5-2m)
                self.state = "turn"
                self.last_switch_time = now

        elif self.state == "turn":
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.9 * self.turn_direction

            if elapsed > 2.0:  # Rotate for 2 seconds (~90 degrees)
                self.state = "forward"
                self.turn_direction *= -1  # Alternate direction
                self.last_switch_time = now

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZigZagDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
