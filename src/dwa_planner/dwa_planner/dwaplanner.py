#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.optimize import minimize
from math import cos, sin, atan2, pi


class DWALeader(Node):
    """
    ROS 2 node implementing a Dynamic Window Approach (DWA) for TurtleBot4 navigation.
    Uses LIDAR for dynamic obstacle avoidance and drives toward a relative goal.
    """

    def __init__(self):
        super().__init__('dwa_leader_node')

        self.declare_parameter('dt', 0.15)
        self.dt = self.get_parameter('dt').value

        self.turn_limit_rad = pi / 12  # 15 degrees
        self.omega_limit = self.turn_limit_rad / self.dt
        self.safe_distance = 0.2
        self.v_max = 2.0
        self.omega_max = pi / 2400

        self.position = np.array([0.0, 0.0])
        self.theta = 0.0
        self.ready = False
        self.goal_offset = np.array([2.5, 0.0])
        self.initial_pose_set = False
        self.goal = np.array([0.0, 0.0])
        self.goal_reached = False
        self.dynamic_obstacles = []
        self.cost_eval_count = 0

        self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/robot1/scan', self.laserscan_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.start_pub = self.create_publisher(PointStamped, '/robot1/start_position', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("DWALeader Node Initialized")

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)
        self.ready = True

        if not self.initial_pose_set:
            self.goal = self.position + self.goal_offset
            start_msg = PointStamped()
            start_msg.header.stamp = self.get_clock().now().to_msg()
            start_msg.header.frame_id = 'odom'
            start_msg.point.x = float(self.position[0])
            start_msg.point.y = float(self.position[1])
            self.start_pub.publish(start_msg)

            self.get_logger().info(
                f"Start: ({self.position[0]:.2f}, {self.position[1]:.2f}) → "
                f"Goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})"
            )
            self.initial_pose_set = True

    def laserscan_callback(self, msg):
        self.dynamic_obstacles.clear()
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                corrected_angle = angle + (pi / 2)
                lx = r * cos(corrected_angle)
                ly = r * sin(corrected_angle)
                gx = self.position[0] + (lx * cos(self.theta) - ly * sin(self.theta))
                gy = self.position[1] + (lx * sin(self.theta) + ly * cos(self.theta))
                self.dynamic_obstacles.append((gx, gy))
            angle += msg.angle_increment

    def control_loop(self):
        if not self.ready or not self.initial_pose_set:
            return

        dist_to_goal = np.linalg.norm(self.goal - self.position)

        if not self.goal_reached and dist_to_goal < 0.3:
            self.goal_reached = True
            self.get_logger().info(f"Goal reached at ({self.position[0]:.2f}, {self.position[1]:.2f})")
            self.send_cmd(0.0, 0.0)
            return

        if self.goal_reached:
            self.send_cmd(0.0, 0.0)
            return

        def constraint_omega_turn(x):
            return self.turn_limit_rad - abs(x[1] * self.dt)

        constraints = [{'type': 'ineq', 'fun': constraint_omega_turn}]

        result = minimize(
            lambda x: -self.dwa_cost(x),
            [0.2, 0.0],
            bounds=[(0.1, self.v_max), (-self.omega_limit, self.omega_limit)],
            constraints=constraints,
            method='SLSQP',
            options={'maxiter': 25}
        )

        v, omega = (result.x if result.success else (0.1, 0.1))

        if dist_to_goal < 1.0:
            v = min(v, dist_to_goal * 0.8)

        self.send_cmd(v, omega)

    def dwa_cost(self, x):
        v, omega = x
        theta_new = self.theta + omega * self.dt
        p_new = self.position + v * np.array([cos(theta_new), sin(theta_new)]) * self.dt

        alpha = 35     # Goal proximity
        beta = 1       # Velocity reward
        delta = 2.5    # Obstacle penalty
        gamma = 2.5    # Heading alignment

        if not self.dynamic_obstacles:
            min_dist = 100.0
        else:
            min_dist = min(np.linalg.norm(p_new - np.array(obs)) for obs in self.dynamic_obstacles)

        goal_cost = -np.linalg.norm(self.goal - p_new)
        velocity_reward = v
        dist_to_obstacle = min_dist - self.safe_distance + 1e-3
        obstacle_cost = -1.0 / dist_to_obstacle

        goal_dir = atan2(self.goal[1] - p_new[1], self.goal[0] - p_new[0])
        heading_diff = abs((goal_dir - theta_new + pi) % (2 * pi) - pi)
        heading_cost = heading_diff

        if min_dist < 0.2:
            delta = 8.0
            if min_dist < 0.1:
                gamma = 0.0

        total_cost = (
            alpha * goal_cost +
            beta * velocity_reward +
            delta * obstacle_cost -
            gamma * heading_cost
        )

        self.cost_eval_count += 1
        if self.cost_eval_count % 10 == 0:
            self.log_cost_breakdown(x, goal_cost, velocity_reward, obstacle_cost, heading_cost, total_cost)

        return total_cost

    def log_cost_breakdown(self, x, goal_cost, velocity_reward, obstacle_cost, heading_cost, total_cost):
        v, omega = x
        self.get_logger().info(
            f"[DWA] v: {v:.2f}, ω: {omega:.2f} | Goal: {goal_cost:.2f}, Vel: {velocity_reward:.2f}, "
            f"Obstacle: {obstacle_cost:.2f}, Heading: {-heading_cost:.2f} => Total: {total_cost:.2f}"
        )

    def send_cmd(self, v, omega):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(omega)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DWALeader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
