#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

from controller_manager_msgs.srv import SwitchController


SERVO_CONTROLLER = "servo_controller"

COMMAND_TOPIC = "/servo_controller/commands"
SAFETY_TOPIC = "/robotarm/safety_stop"


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")

        self.safety_stop = False
        self.controller_deactivated = False

        self.sub = self.create_subscription(
            Bool,
            SAFETY_TOPIC,
            self.safety_callback,
            10,
        )

        self.zero_pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        self.switch_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
        )

        self.timer = self.create_timer(
            0.1,
            self.update,
        )

        self.get_logger().info("Safety supervisor started")

    def safety_callback(self, msg):
        self.safety_stop = msg.data

    def publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.zero_pub.publish(msg)

    def switch_servo_controller(self, activate):
        if not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "Controller manager switch service not available"
            )
            return

        req = SwitchController.Request()

        if activate:
            req.activate_controllers = [SERVO_CONTROLLER]
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = [SERVO_CONTROLLER]

        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout.sec = 2
        req.timeout.nanosec = 0

        future = self.switch_client.call_async(req)

        action = "activate" if activate else "deactivate"

        future.add_done_callback(
            lambda f: self.get_logger().warn(
                f"{action} {SERVO_CONTROLLER}: "
                f"{f.result().ok if f.result() else False}"
            )
        )

    def deactivate_servo_controller(self):
        if self.controller_deactivated:
            return

        self.get_logger().error(
            "Safety stop active: zeroing commands and deactivating servo_controller"
        )

        for _ in range(10):
            self.publish_zero()

        self.switch_servo_controller(activate=False)
        self.controller_deactivated = True

    def reactivate_servo_controller(self):
        if not self.controller_deactivated:
            return

        self.get_logger().warn(
            "Safety cleared: reactivating servo_controller"
        )

        for _ in range(10):
            self.publish_zero()

        self.switch_servo_controller(activate=True)

        self.controller_deactivated = False

    def update(self):
        if self.safety_stop:
            self.publish_zero()
            self.deactivate_servo_controller()
            return

        self.reactivate_servo_controller()


def main():
    rclpy.init()
    node = SafetySupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
