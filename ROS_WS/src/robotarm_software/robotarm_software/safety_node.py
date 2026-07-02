#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import String

import lgpio


GPIO_CHIP = 4

EMERGENCY_GPIO = 6
RESUME_GPIO = 17

USE_DISTANCE_SENSOR = False
MIN_DISTANCE_M = 0.15


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        self.safety_pub = self.create_publisher(
            Bool,
            "/robotarm/safety_stop",
            10,
        )

        self.reason_pub = self.create_publisher(
            String,
            "/robotarm/safety_reason",
            10,
        )

        self.gpio_handle = lgpio.gpiochip_open(GPIO_CHIP)

        # Emergency is fail-safe:
        # LOW  = safe, cable/switch connected to GND
        # HIGH = stop, button open/pressed/cable disconnected
        lgpio.gpio_claim_input(
            self.gpio_handle,
            EMERGENCY_GPIO,
            lgpio.SET_PULL_UP,
        )

        # Resume button:
        # LOW  = pressed
        # HIGH = not pressed
        lgpio.gpio_claim_input(
            self.gpio_handle,
            RESUME_GPIO,
            lgpio.SET_PULL_UP,
        )

        self.safety_latched = False

        self.last_safety_stop = None
        self.last_reason = ""

        self.timer = self.create_timer(
            0.02,
            self.update,
        )

        self.get_logger().info("Safety node started")
        self.get_logger().info("GPIO6 HIGH = emergency stop")
        self.get_logger().info("GPIO17 LOW = resume")

    def read_emergency_stop(self):
        value = lgpio.gpio_read(
            self.gpio_handle,
            EMERGENCY_GPIO,
        )

        return value == 1

    def read_resume_button(self):
        value = lgpio.gpio_read(
            self.gpio_handle,
            RESUME_GPIO,
        )

        return value == 1

    def read_distance_sensor_stop(self):
        # Placeholder for later distance sensor integration.
        return False

    def update(self):
        emergency_stop = self.read_emergency_stop()
        resume_pressed = self.read_resume_button()
        distance_stop = self.read_distance_sensor_stop()

        reasons = []

        if emergency_stop:
            self.safety_latched = True
            reasons.append("emergency_button_or_cable")

        if distance_stop:
            self.safety_latched = True
            reasons.append("distance_too_small")

        if self.safety_latched and resume_pressed:
            if not emergency_stop and not distance_stop:
                self.safety_latched = False
                reasons = []
                self.get_logger().warn("Resume pressed: safety stop cleared")
            else:
                reasons.append("resume_ignored_fault_still_active")

        safety_stop = self.safety_latched

        if safety_stop and not reasons:
            reasons.append("latched_stop_waiting_for_resume")

        reason_text = ",".join(reasons) if reasons else "ok"

        safety_msg = Bool()
        safety_msg.data = safety_stop
        self.safety_pub.publish(safety_msg)

        reason_msg = String()
        reason_msg.data = reason_text
        self.reason_pub.publish(reason_msg)

        if safety_stop != self.last_safety_stop or reason_text != self.last_reason:
            if safety_stop:
                self.get_logger().error(
                    f"SAFETY STOP ACTIVE: {reason_text}"
                )
            else:
                self.get_logger().warn("Safety clear")

            self.last_safety_stop = safety_stop
            self.last_reason = reason_text

    def destroy_node(self):
        for pin in [EMERGENCY_GPIO, RESUME_GPIO]:
            try:
                lgpio.gpio_free(
                    self.gpio_handle,
                    pin,
                )
            except Exception:
                pass

        try:
            lgpio.gpiochip_close(
                self.gpio_handle,
            )
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = SafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
