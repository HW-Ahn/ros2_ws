# tb3_security_system/nodes/seq_patrol.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class SequencePatrol(Node):
    def __init__(self):
        super().__init__('seq_patrol')

        self.points = [
            (-1.8, -2.2, 0.0),
            ( 1.8, -2.2, math.pi),
            ( 1.8,  2.2, math.pi/2),
            (-1.8,  2.2, -math.pi/2),
        ]
        self.index = 0

        self.pub = self.create_publisher(PoseStamped, "/tb3_1/cmd_goal", 10)

        self.timer = self.create_timer(12.0, self.send_next)
        self.get_logger().info("[SeqPatrol] Started")

    def send_next(self):
        x, y, yaw = self.points[self.index]

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw/2)
        goal.pose.orientation.w = math.cos(yaw/2)

        self.pub.publish(goal)
        self.get_logger().info(f"[SeqPatrol] Going to QR{self.index+1}")

        self.index = (self.index + 1) % len(self.points)

def main():
    rclpy.init()
    node = SequencePatrol()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
