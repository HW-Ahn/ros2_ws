# tb3_security_system/nodes/return_home.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class ReturnHome(Node):
    def __init__(self):
        super().__init__('return_home')

        self.home_pose = (-2.4, -0.4, 0.0)

        self.sub = self.create_subscription(
            Odometry,
            "/tb3_1/odom",
            self.odom_cb,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            "/tb3_1/cmd_goal",
            10
        )

        self.sent = False

    def odom_cb(self, msg):
        if self.sent:
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = self.home_pose[0]
        ps.pose.position.y = self.home_pose[1]
        ps.pose.orientation.w = 1.0

        self.pub.publish(ps)

        self.get_logger().info("[ReturnHome] Returning to (0,0)")
        self.sent = True

def main():
    rclpy.init()
    node = ReturnHome()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
