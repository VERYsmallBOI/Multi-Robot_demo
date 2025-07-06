#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class RobotMimic(Node):
    def __init__(self):
        super().__init__('robot_mimic')

        self.sub = self.create_subscription(
            TwistStamped,
            '/robot2/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            '/robot1/cmd_vel',
            10
        )

        self.get_logger().info('Mimic node active: forwarding /robot2/cmd_vel to /robot1/cmd_vel')

    def cmd_callback(self, msg: TwistStamped):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMimic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
