# tb3_security_system/nodes/random_patrol.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import math

class RandomPatrol(Node):
    def __init__(self):
        super().__init__('random_patrol')

        self.pub = self.create_publisher(PoseStamped, "/tb3_1/cmd_goal", 10)

        self.timer = self.create_timer(10.0, self.send_random_goal)
        self.get_logger().info("[RandomPatrol] Started")

    def send_random_goal(self):
        x = random.uniform(-2.0, 2.0)
        y = random.uniform(-2.0, 2.0)
        yaw = random.uniform(-math.pi, math.pi)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw/2)
        goal.pose.orientation.w = math.cos(yaw/2)

        self.pub.publish(goal)
        self.get_logger().info(f"[RandomPatrol] → ({x:.2f}, {y:.2f})")

def main():
    rclpy.init()
    node = RandomPatrol()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
