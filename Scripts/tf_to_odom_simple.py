#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class TfToOdomSimple(Node):
    def __init__(self):
        super().__init__('tf_to_odom_simple')
        # 1) Subscribe to the raw /tf stream
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_cb,
            10)

        # 2) Publish Odometry on /odom_est
        self.odom_pub = self.create_publisher(Odometry, '/odom_est', 10)
        self.get_logger().info("Listening on /tf, publishing /odom_est")

    def tf_cb(self, msg: TFMessage):
        if not msg.transforms:
            return

        # 3) Grab the *first* transform
        t = msg.transforms[0]

        # 4) Fill an Odometry message
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'              # or 'world' if you prefer
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = t.transform.translation.z
        odom.pose.pose.orientation    = t.transform.rotation

        # leave twist zero for now
        odom.twist.twist.linear.x  = 0.0
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = 0.0

        # 5) Publish
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = TfToOdomSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

