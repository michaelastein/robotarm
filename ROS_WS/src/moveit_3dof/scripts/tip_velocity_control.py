#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


BASE_FRAME = "base_link"
TIP_FRAME = "tool_tip_link"

PUBLISH_PERIOD = 0.01

MAX_VEL = 0.04


def clamp(value, low, high):
    return max(low, min(high, value))


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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.switch_to_twist()

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_twist,
        )

        self.get_logger().info(
            "Input format: vx vy vz | example: 0.01 0 0 | "
            "s stop | p print tip | q quit"
        )
        self.get_logger().info(
            "No height guard, no correction. Publishing raw Cartesian twist."
        )

    def switch_to_twist(self):
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo...")

        if not rclpy.ok():
            return

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        if future.result() and future.result().success:
            self.get_logger().info("Servo switched to TWIST mode")
        else:
            self.get_logger().error("Could not switch Servo to TWIST mode")

    def get_tip_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                TIP_FRAME,
                rclpy.time.Time(),
            )

            t = transform.transform.translation

            return (
                float(t.x),
                float(t.y),
                float(t.z),
            )

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get TF {BASE_FRAME} -> {TIP_FRAME}: {ex}",
                throttle_duration_sec=1.0,
            )
            return None

    def print_tip_position(self):
        pos = self.get_tip_position()

        if pos is None:
            print("Tip position unknown")
            return

        x, y, z = pos

        print(
            f"tip x={x:+.4f} y={y:+.4f} z={z:+.4f} | "
            f"cmd vx={self.vx:+.3f} vy={self.vy:+.3f} vz={self.vz:+.3f}"
        )

    def publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)

    def publish_twist(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME

        msg.twist.linear.x = clamp(self.vx, -MAX_VEL, MAX_VEL)
        msg.twist.linear.y = clamp(self.vy, -MAX_VEL, MAX_VEL)
        msg.twist.linear.z = clamp(self.vz, -MAX_VEL, MAX_VEL)

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)

    def input_loop(self):
        while rclpy.ok():
            text = input(
                "\nEnter vx vy vz (s stop, p print tip, q quit): "
            ).strip()

            if text == "q":
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.publish_zero()
                rclpy.shutdown()
                return

            if text == "s":
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.publish_zero()
                print("Stopped")
                continue

            if text == "p":
                self.print_tip_position()
                continue

            try:
                vals = [float(v) for v in text.split()]

                if len(vals) != 3:
                    print("Need exactly 3 numbers: vx vy vz")
                    print("Example: 0.01 0 0")
                    continue

                self.vx = clamp(vals[0], -MAX_VEL, MAX_VEL)
                self.vy = clamp(vals[1], -MAX_VEL, MAX_VEL)
                self.vz = clamp(vals[2], -MAX_VEL, MAX_VEL)

                print(
                    f"Sending vx={self.vx:+.3f}, "
                    f"vy={self.vy:+.3f}, "
                    f"vz={self.vz:+.3f}"
                )

            except Exception:
                print("Example: 0.01 0 0")


def main():
    rclpy.init()

    node = TipVelocityControl()

    threading.Thread(
        target=node.input_loop,
        daemon=True,
    ).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_zero()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
