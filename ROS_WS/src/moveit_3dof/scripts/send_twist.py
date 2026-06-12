#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


class ServoTwistTest(Node):
    def __init__(self):
        super().__init__("servo_twist_test")

        self.cli = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )

        self.pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        self.timer = self.create_timer(0.05, self.publish_twist)
        self.switched = False

    def switch_to_twist(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Servo command-type service...")

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f"Switch result: {future.result().success}")
        self.switched = True

    def publish_twist(self):
        if not self.switched:
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.01

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ServoTwistTest()
    node.switch_to_twist()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
