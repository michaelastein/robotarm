#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


class TipVelocityControl(Node):

    def __init__(self):
        super().__init__("tip_velocity_control")

        self.pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        self.client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0

        self.switch_to_twist()

        self.timer = self.create_timer(
            0.01,
            self.publish_twist,
        )

    def switch_to_twist(self):

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo...")

        req = ServoCommandType.Request()
        req.command_type = 1

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        print("Servo switched to TWIST mode")

    def publish_twist(self):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz

        msg.twist.angular.x = self.wx
        msg.twist.angular.y = self.wy
        msg.twist.angular.z = self.wz

        self.pub.publish(msg)

    def input_loop(self):

        while rclpy.ok():

            text = input(
                "\nEnter vx vy vz wx wy wz (q to quit): "
            )

            if text.strip() == "q":
                rclpy.shutdown()
                return

            try:

                vals = [float(v) for v in text.split()]

                if len(vals) != 6:
                    print("Need exactly 6 numbers")
                    continue

                self.vx = vals[0]
                self.vy = vals[1]
                self.vz = vals[2]

                self.wx = vals[3]
                self.wy = vals[4]
                self.wz = vals[5]

                print(
                    "Sending:",
                    vals,
                )

            except Exception:
                print("Example: 0 0 0.1 0 0 0")


def main():

    rclpy.init()

    node = TipVelocityControl()

    threading.Thread(
        target=node.input_loop,
        daemon=True,
    ).start()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
