#!/usr/bin/env python3
import os
import pickle
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def normalize_angle(angle):
    """Wrap to [-pi, pi)."""
    while angle > math.pi:
        angle -= 2*math.pi
    while angle <= -math.pi:
        angle += 2*math.pi
    return angle

class GoToRockSimple(Node):
    def __init__(self):
        super().__init__('go_to_rock_simple')
        # load rock DB
        db_path = os.path.expanduser('~/rock_features.db')
        if not os.path.isfile(db_path):
            self.get_logger().error(f"DB not found: {db_path}")
            raise RuntimeError("rock_features.db missing")
        with open(db_path, 'rb') as f:
            self.db = pickle.load(f)
        self.get_logger().info(f"Loaded {len(self.db)} rocks from DB")

        # parameters
        self.declare_parameter('rock_id', 1)
        self.declare_parameter('kp_lin',   0.5)
        self.declare_parameter('kp_ang',   2.0)
        self.declare_parameter('max_lin',  0.3)
        self.declare_parameter('max_ang',  1.0)
        self.declare_parameter('tolerance',0.2)

        rock_id = self.get_parameter('rock_id').value
        if rock_id < 0 or rock_id >= len(self.db):
            self.get_logger().error(f"Invalid rock_id: {rock_id}")
            raise RuntimeError("rock_id out of range")
        rock = self.db[rock_id]
        self.goal_x, self.goal_y = rock['position'][0], rock['position'][1]
        self.get_logger().info(
            f"Navigating to rock#{rock_id} at x:{self.goal_x:.2f}, y:{self.goal_y:.2f}"
        )

        self.kp_lin = self.get_parameter('kp_lin').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.max_lin= self.get_parameter('max_lin').value
        self.max_ang= self.get_parameter('max_ang').value
        self.tol    = self.get_parameter('tolerance').value

        # state
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False

        # pub/sub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/odom_est', self.odom_cb, 10)

        # timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.have_odom = True

    def control_loop(self):
        if not self.have_odom:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        twist = Twist()
        if dist < self.tol:
            self.get_logger().info("🚩 Arrived at rock, stopping")
            self.pub.publish(twist)
            self.timer.cancel()
            return

        # heading
        target_yaw = math.atan2(dy, dx)
        err_yaw = normalize_angle(target_yaw - self.yaw)

        # P-controller
        v = self.kp_lin * dist
        w = self.kp_ang * err_yaw

        # clamp
        twist.linear.x  = max(-self.max_lin, min(self.max_lin, v))
        twist.angular.z = max(-self.max_ang, min(self.max_ang, w))

        self.pub.publish(twist)

    def destroy_node(self):
        # ensure cmd_vel zeroed on exit
        stop = Twist()
        self.pub.publish(stop)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GoToRockSimple()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(str(e))
        rclpy.shutdown()
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':
    main()

