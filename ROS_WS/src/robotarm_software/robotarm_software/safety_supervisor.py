#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


SERVO_CONTROLLER = "servo_controller"
TRAJECTORY_CONTROLLER = "arm_trajectory_controller"

MOTION_CONTROLLERS = {
    SERVO_CONTROLLER,
    TRAJECTORY_CONTROLLER,
}

SERVO_COMMAND_TOPIC = "/servo_controller/commands"
SAFETY_TOPIC = "/robotarm/safety_stop"

LIST_CONTROLLERS_SERVICE = "/controller_manager/list_controllers"
SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"

UPDATE_PERIOD = 0.1
SERVICE_TIMEOUT = 1.0
SWITCH_TIMEOUT_SECONDS = 2

ZERO_COMMAND_COUNT = 10


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__("safety_supervisor")

        self.lock = threading.Lock()

        self.safety_stop = False

        # Prevent repeated service requests.
        self.operation_in_progress = False

        # True after all active motion controllers were deactivated.
        self.controllers_deactivated = False

        # Controllers that were active before the safety stop.
        self.controllers_to_reactivate = []

        self.safety_sub = self.create_subscription(
            Bool,
            SAFETY_TOPIC,
            self.safety_callback,
            10,
        )

        # Immediate zero command for the direct velocity Servo mode.
        self.zero_pub = self.create_publisher(
            Float64MultiArray,
            SERVO_COMMAND_TOPIC,
            10,
        )

        self.list_client = self.create_client(
            ListControllers,
            LIST_CONTROLLERS_SERVICE,
        )

        self.switch_client = self.create_client(
            SwitchController,
            SWITCH_CONTROLLER_SERVICE,
        )

        self.timer = self.create_timer(
            UPDATE_PERIOD,
            self.update,
        )

        self.get_logger().info("Safety supervisor started")
        self.get_logger().info(
            "Protected controllers: "
            f"{SERVO_CONTROLLER}, {TRAJECTORY_CONTROLLER}"
        )

    # ========================================================
    # Safety state
    # ========================================================

    def safety_callback(self, msg):
        with self.lock:
            self.safety_stop = bool(msg.data)

    def get_safety_stop(self):
        with self.lock:
            return self.safety_stop

    # ========================================================
    # Zero commands
    # ========================================================

    def publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.zero_pub.publish(msg)

    def publish_zero_repeatedly(self):
        for _ in range(ZERO_COMMAND_COUNT):
            self.publish_zero()

    # ========================================================
    # Controller listing
    # ========================================================

    def request_active_motion_controllers(self):
        if self.operation_in_progress:
            return

        if not self.list_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT
        ):
            self.get_logger().error(
                "list_controllers service is unavailable"
            )
            return

        self.operation_in_progress = True

        request = ListControllers.Request()
        future = self.list_client.call_async(request)
        future.add_done_callback(
            self.active_controller_list_received
        )

    def active_controller_list_received(self, future):
        try:
            response = future.result()

            if response is None:
                self.get_logger().error(
                    "No response from list_controllers"
                )
                self.operation_in_progress = False
                return

            active_motion_controllers = [
                controller.name
                for controller in response.controller
                if (
                    controller.name in MOTION_CONTROLLERS
                    and controller.state == "active"
                )
            ]

            self.controllers_to_reactivate = list(
                active_motion_controllers
            )

            if not active_motion_controllers:
                self.get_logger().warn(
                    "No active motion controller found"
                )
                self.controllers_deactivated = True
                self.operation_in_progress = False
                return

            self.get_logger().error(
                "Deactivating active motion controller(s): "
                + ", ".join(active_motion_controllers)
            )

            # Keep operation_in_progress True while switching.
            self.send_switch_request(
                activate_controllers=[],
                deactivate_controllers=active_motion_controllers,
                completion_callback=self.deactivation_finished,
            )

        except Exception as exception:
            self.get_logger().error(
                f"Failed to list controllers: {exception}"
            )
            self.operation_in_progress = False

    # ========================================================
    # Controller switching
    # ========================================================

    def send_switch_request(
        self,
        activate_controllers,
        deactivate_controllers,
        completion_callback,
    ):
        if not self.switch_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT
        ):
            self.get_logger().error(
                "switch_controller service is unavailable"
            )
            self.operation_in_progress = False
            return

        request = SwitchController.Request()

        request.activate_controllers = list(
            activate_controllers
        )
        request.deactivate_controllers = list(
            deactivate_controllers
        )

        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = SWITCH_TIMEOUT_SECONDS
        request.timeout.nanosec = 0

        future = self.switch_client.call_async(request)

        future.add_done_callback(
            lambda completed_future: self.switch_request_finished(
                completed_future,
                activate_controllers,
                deactivate_controllers,
                completion_callback,
            )
        )

    def switch_request_finished(
        self,
        future,
        activate_controllers,
        deactivate_controllers,
        completion_callback,
    ):
        success = False

        try:
            response = future.result()

            if response is not None:
                success = bool(response.ok)

            if success:
                self.get_logger().warn(
                    "Controller switch successful: "
                    f"activate={list(activate_controllers)}, "
                    f"deactivate={list(deactivate_controllers)}"
                )
            else:
                self.get_logger().error(
                    "Controller switch failed: "
                    f"activate={list(activate_controllers)}, "
                    f"deactivate={list(deactivate_controllers)}"
                )

        except Exception as exception:
            self.get_logger().error(
                f"Controller switch exception: {exception}"
            )

        self.operation_in_progress = False
        completion_callback(success)

    # ========================================================
    # Safety stop
    # ========================================================

    def begin_safety_stop(self):
        if self.controllers_deactivated:
            return

        if self.operation_in_progress:
            return

        self.get_logger().error(
            "Safety stop active: stopping all robot motion"
        )

        self.publish_zero_repeatedly()
        self.request_active_motion_controllers()

    def deactivation_finished(self, success):
        if success:
            self.controllers_deactivated = True

            self.get_logger().error(
                "Motion controller deactivated"
            )
        else:
            self.controllers_deactivated = False

            self.get_logger().error(
                "Motion controller deactivation failed; "
                "will retry while safety stop remains active"
            )

    # ========================================================
    # Resume
    # ========================================================

    def begin_safety_clear(self):
        if not self.controllers_deactivated:
            return

        if self.operation_in_progress:
            return

        if not self.controllers_to_reactivate:
            self.get_logger().warn(
                "Safety cleared; no controller needs reactivation"
            )

            self.controllers_deactivated = False
            return

        controllers = list(self.controllers_to_reactivate)

        self.get_logger().warn(
            "Safety cleared: reactivating previous controller(s): "
            + ", ".join(controllers)
        )

        self.publish_zero_repeatedly()
        self.operation_in_progress = True

        self.send_switch_request(
            activate_controllers=controllers,
            deactivate_controllers=[],
            completion_callback=self.reactivation_finished,
        )

    def reactivation_finished(self, success):
        if success:
            self.get_logger().warn(
                "Previous motion controller reactivated"
            )

            self.controllers_to_reactivate = []
            self.controllers_deactivated = False
        else:
            self.get_logger().error(
                "Controller reactivation failed; will retry"
            )

    # ========================================================
    # Main loop
    # ========================================================

    def update(self):
        safety_stop = self.get_safety_stop()

        if safety_stop:
            # Keep sending zeros for the Servo controller while stopped.
            self.publish_zero()
            self.begin_safety_stop()
        else:
            self.begin_safety_clear()


def main(args=None):
    rclpy.init(args=args)

    node = SafetySupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_repeatedly()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
