#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


BASE_FRAME = "base_link"
TIP_FRAME = "tool_tip_link"

PUBLISH_PERIOD = 0.01

DEADBAND = 0.12

KP_X = 0.010
KP_Y = 0.0

MAX_VEL = 0.015

SMOOTHING_ALPHA = 0.20
LOST_ALPHA = 0.35

COMMAND_EPSILON = 0.00002
COMMAND_MULTIPLIER = 3.0

TARGET_TIMEOUT = 0.25
STOP_COMMANDS_AFTER_LOST = 5

DEBUG_PRINT_PERIOD = 1.0


def clamp(value, low, high):
    return max(low, min(high, value))


class HotspotServo(Node):

    def __init__(self):
        super().__init__("hotspot_servo")

        self.target_sub = self.create_subscription(
            Float32MultiArray,
            "/hotspot/target",
            self.target_callback,
            10,
        )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        self.servo_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.lock = threading.Lock()

        self.target_visible = False
        self.err_x = 0.0
        self.err_y = 0.0
        self.conf = 0.0
        self.last_target_time = None

        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0

        self.lost_stop_publish_count = STOP_COMMANDS_AFTER_LOST

        self.last_debug_time = 0.0
        self.last_tip_pos = None

        self.switch_to_twist()

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_twist,
        )

        self.get_logger().info("Hotspot servo started")
        self.get_logger().info("Listening to /hotspot/target")
        self.get_logger().info("Publishing to /servo_node/delta_twist_cmds")
        self.get_logger().warn(
            "DEBUG MODE: only err_x controls base_link linear.y. "
            "linear.x=0 and linear.z=0."
        )

    def switch_to_twist(self):
        while rclpy.ok() and not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo...")

        if not rclpy.ok():
            return

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST

        future = self.servo_client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        if future.result() and future.result().success:
            self.get_logger().info("Servo switched to TWIST mode")
        else:
            self.get_logger().error("Could not switch Servo to TWIST mode")

    def target_callback(self, msg):
        if len(msg.data) < 4:
            return

        with self.lock:
            self.target_visible = msg.data[0] >= 0.5
            self.err_x = float(msg.data[1])
            self.err_y = float(msg.data[2])
            self.conf = float(msg.data[3])
            self.last_target_time = time.monotonic()

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

        except TransformException:
            return None

    def create_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE_FRAME

        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        return twist

    def publish_zero_once(self):
        self.twist_pub.publish(self.create_zero_twist())

    def maybe_debug_print(self, did_publish=False, reason=""):
        now = time.monotonic()

        if now - self.last_debug_time < DEBUG_PRINT_PERIOD:
            return

        self.last_debug_time = now

        tip_pos = self.get_tip_position()

        if tip_pos is None:
            tip_text = "tip=unknown"
            delta_text = "d=?"
        else:
            x, y, z = tip_pos

            if self.last_tip_pos is None:
                dx = 0.0
                dy = 0.0
                dz = 0.0
            else:
                dx = x - self.last_tip_pos[0]
                dy = y - self.last_tip_pos[1]
                dz = z - self.last_tip_pos[2]

            self.last_tip_pos = tip_pos

            tip_text = f"tip x={x:+.4f} y={y:+.4f} z={z:+.4f}"
            delta_text = f"d={dx:+.4f},{dy:+.4f},{dz:+.4f}"

        self.get_logger().info(
            f"{tip_text} | {delta_text} | "
            f"visible={self.target_visible} "
            f"err=({self.err_x:+.3f},{self.err_y:+.3f}) "
            f"conf={self.conf:.2f} | "
            f"filtered=({self.filtered_vx:+.5f},"
            f"{self.filtered_vy:+.5f},"
            f"{self.filtered_vz:+.5f}) | "
            f"cmd=({self.cmd_vx:+.5f},"
            f"{self.cmd_vy:+.5f},"
            f"{self.cmd_vz:+.5f}) | "
            f"published={did_publish} reason={reason}"
        )

    def target_is_fresh(self):
        if self.last_target_time is None:
            return False

        return (time.monotonic() - self.last_target_time) <= TARGET_TIMEOUT

    def publish_twist(self):
        with self.lock:
            target_visible = self.target_visible
            err_x = self.err_x
            err_y = self.err_y
            conf = self.conf
            fresh = self.target_is_fresh()

        active = target_visible and fresh

        raw_vx = 0.0
        raw_vy = 0.0
        raw_vz = 0.0

        alpha = SMOOTHING_ALPHA if active else LOST_ALPHA

        if active:
            if abs(err_x) < DEADBAND:
                err_x = 0.0

            # DEBUG MODE:
            # Only image-x error controls base_link linear.y.
            raw_vx = 0.0
            raw_vy = KP_X * err_x
            raw_vz = 0.0

            raw_vy = clamp(raw_vy, -MAX_VEL, MAX_VEL)

        self.filtered_vx = (
            alpha * raw_vx
            + (1.0 - alpha) * self.filtered_vx
        )

        self.filtered_vy = (
            alpha * raw_vy
            + (1.0 - alpha) * self.filtered_vy
        )

        self.filtered_vz = 0.0

        cmd_vx = clamp(
            self.filtered_vx * COMMAND_MULTIPLIER,
            -MAX_VEL,
            MAX_VEL,
        )

        cmd_vy = clamp(
            self.filtered_vy * COMMAND_MULTIPLIER,
            -MAX_VEL,
            MAX_VEL,
        )

        cmd_vz = 0.0

        if abs(cmd_vx) < COMMAND_EPSILON:
            cmd_vx = 0.0

        if abs(cmd_vy) < COMMAND_EPSILON:
            cmd_vy = 0.0

        if abs(cmd_vz) < COMMAND_EPSILON:
            cmd_vz = 0.0

        self.cmd_vx = cmd_vx
        self.cmd_vy = cmd_vy
        self.cmd_vz = cmd_vz

        if not active:
            self.filtered_vx = 0.0
            self.filtered_vy = 0.0
            self.filtered_vz = 0.0

            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = 0.0

            if self.lost_stop_publish_count < STOP_COMMANDS_AFTER_LOST:
                self.publish_zero_once()
                self.lost_stop_publish_count += 1
                self.maybe_debug_print(
                    did_publish=True,
                    reason="lost_or_stale_zero_stop",
                )
            else:
                self.maybe_debug_print(
                    did_publish=False,
                    reason="lost_or_stale_no_publish",
                )

            return

        if (
            abs(cmd_vx) < COMMAND_EPSILON
            and abs(cmd_vy) < COMMAND_EPSILON
            and abs(cmd_vz) < COMMAND_EPSILON
        ):
            self.maybe_debug_print(
                did_publish=False,
                reason="active_but_tiny_no_publish",
            )
            return

        self.lost_stop_publish_count = 0

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE_FRAME

        twist.twist.linear.x = cmd_vx
        twist.twist.linear.y = cmd_vy
        twist.twist.linear.z = cmd_vz

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        self.twist_pub.publish(twist)

        self.maybe_debug_print(
            did_publish=True,
            reason="active_tracking",
        )


def main(args=None):
    rclpy.init(args=args)

    node = HotspotServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_zero_once()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
