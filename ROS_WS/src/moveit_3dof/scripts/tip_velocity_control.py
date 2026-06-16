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
TIP_FRAME = "lower_arm_link"

PUBLISH_PERIOD = 0.01

KP_Z = 1.2
MAX_VZ = 0.06
Z_DEADBAND = 0.003


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
        self.vz_correction = 0.0

        self.target_z = None

        self.switch_to_twist()

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_twist,
        )

        self.get_logger().info(
            "Input format: vx vy   | example: 0 0.05   | q to quit"
        )

    def switch_to_twist(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo...")

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

    def get_tip_z(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                TIP_FRAME,
                rclpy.time.Time(),
            )

            return transform.transform.translation.z

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not get TF {BASE_FRAME} -> {TIP_FRAME}: {ex}",
                throttle_duration_sec=1.0,
            )
            return None

    def compute_z_correction(self):
        current_z = self.get_tip_z()

        if current_z is None:
            return 0.0

        if self.target_z is None:
            self.target_z = current_z
            self.get_logger().info(
                f"Locked target height z = {self.target_z:.4f} m"
            )
            return 0.0

        z_error = self.target_z - current_z

        if abs(z_error) < Z_DEADBAND:
            return 0.0

        vz = KP_Z * z_error
        vz = clamp(
            vz,
            -MAX_VZ,
            MAX_VZ,
        )

        return vz

    def publish_twist(self):
        self.vz_correction = self.compute_z_correction()

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME

        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz_correction

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)

    def input_loop(self):
        while rclpy.ok():
            text = input(
                "\nEnter vx vy (q quit, z reset height): "
            ).strip()

            if text == "q":
                rclpy.shutdown()
                return

            if text == "z":
                self.target_z = None
                print("Height target reset to current tip height")
                continue

            try:
                vals = [float(v) for v in text.split()]

                if len(vals) != 2:
                    print("Need exactly 2 numbers: vx vy")
                    print("Example: 0 0.05")
                    continue

                self.vx = vals[0]
                self.vy = vals[1]

                print(
                    f"Sending vx={self.vx:.3f}, "
                    f"vy={self.vy:.3f}, "
                    f"auto vz={self.vz_correction:.3f}"
                )

            except Exception:
                print("Example: 0 0.05")


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
