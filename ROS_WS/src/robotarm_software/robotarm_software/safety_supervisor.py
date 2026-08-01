#!/usr/bin/env python3
"""
ROS 2 safety supervisor for stopping and restoring robot motion controllers.

The supervisor listens to a latched safety-stop topic. When a stop becomes
active, it immediately publishes zero joint-velocity commands and deactivates
any protected motion controller that is currently active.

When the safety stop clears, the supervisor reactivates only the controllers
that were active before the stop.

Protected controllers:
    SERVO_CONTROLLER:
        Direct joint-velocity controller used by the hotspot servo node.

    TRAJECTORY_CONTROLLER:
        Trajectory controller used for planned arm motion.

ROS topics:
    SAFETY_TOPIC:
        std_msgs/msg/Bool.

        True:
            Immediately stop motion and deactivate active protected controllers.

        False:
            Reactivate controllers that were active before the safety stop.

    SERVO_COMMAND_TOPIC:
        std_msgs/msg/Float64MultiArray containing joint velocity commands in
        the order expected by the servo controller. The supervisor publishes
        [0.0, 0.0, 0.0] during a stop and before controller transitions.

Controller-manager services:
    LIST_CONTROLLERS_SERVICE:
        Lists available controllers and their current states.

    SWITCH_CONTROLLER_SERVICE:
        Activates or deactivates selected controllers.

Timing and retry parameters:
    UPDATE_PERIOD_SECONDS:
        Period of the supervisor timer. A value of 0.1 seconds corresponds to
        10 Hz.

    SERVICE_TIMEOUT_SECONDS:
        Maximum time spent waiting for a controller-manager service to become
        available during one attempt.

    SWITCH_TIMEOUT_SECONDS:
        Timeout included in each controller switch request.

    ZERO_COMMAND_COUNT:
        Number of zero-velocity commands published before controller switching
        and during final shutdown.

Safety behavior:
    - Zero velocity is published immediately while safety is active.
    - Only controllers that were active before the stop are remembered.
    - Repeated service calls are prevented by operation_in_progress.
    - Failed deactivation or reactivation attempts are retried by the timer.
"""

import threading
from collections.abc import Callable, Iterable
from typing import Optional

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray


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

UPDATE_PERIOD_SECONDS = 0.1
SERVICE_TIMEOUT_SECONDS = 1.0
SWITCH_TIMEOUT_SECONDS = 2

ZERO_COMMAND_COUNT = 10


class SafetySupervisor(Node):
    """Coordinate safety-stop commands and motion-controller state changes."""

    def __init__(self) -> None:
        """
        Initialize ROS interfaces and internal controller-state tracking.

        Parameters:
            None.

        Returns:
            None.
        """
        super().__init__("safety_supervisor")

        self.lock = threading.Lock()

        self.safety_stop = False
        self.operation_in_progress = False
        self.controllers_deactivated = False
        self.controllers_to_reactivate: list[str] = []

        self.safety_sub = self.create_subscription(
            Bool,
            SAFETY_TOPIC,
            self.safety_callback,
            10,
        )

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
            UPDATE_PERIOD_SECONDS,
            self.update,
        )

        self.get_logger().info("Safety supervisor started")

    def safety_callback(self, msg: Bool) -> None:
        """
        Store the latest safety-stop state.

        Parameters:
            msg:
                Bool message from SAFETY_TOPIC. True requests an immediate
                motion stop; False permits controller restoration.

        Returns:
            None.
        """
        with self.lock:
            self.safety_stop = bool(msg.data)

    def get_safety_stop(self) -> bool:
        """
        Read the current safety-stop state under the thread lock.

        Parameters:
            None.

        Returns:
            True when the safety stop is active; otherwise False.
        """
        with self.lock:
            return self.safety_stop

    def publish_zero(self) -> None:
        """
        Publish one zero joint-velocity command.

        Parameters:
            None.

        Returns:
            None.
        """
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.zero_pub.publish(msg)

    def publish_zero_repeatedly(self) -> None:
        """
        Publish several zero joint-velocity commands in immediate succession.

        Repetition increases the chance that the velocity controller receives
        a stop command before it is deactivated or before the process exits.

        Parameters:
            None.

        Returns:
            None.
        """
        for _ in range(ZERO_COMMAND_COUNT):
            self.publish_zero()

    def request_active_motion_controllers(self) -> None:
        """
        Request the controller-manager list and identify active motion controllers.

        The asynchronous response is handled by
        active_controller_list_received().

        Parameters:
            None.

        Returns:
            None.
        """
        if self.operation_in_progress:
            return

        if not self.list_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT_SECONDS
        ):
            self.get_logger().error(
                "list_controllers service is unavailable"
            )
            return

        self.operation_in_progress = True

        request = ListControllers.Request()
        future = self.list_client.call_async(request)
        future.add_done_callback(self.active_controller_list_received)

    def active_controller_list_received(self, future) -> None:
        """
        Process the controller list and begin deactivating active motion controllers.

        Parameters:
            future:
                Completed asynchronous ListControllers service future.

        Returns:
            None.
        """
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
                self.controllers_deactivated = True
                self.operation_in_progress = False
                return

            self.get_logger().error(
                "Deactivating active motion controller(s): "
                + ", ".join(active_motion_controllers)
            )

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

    def send_switch_request(
        self,
        activate_controllers: Iterable[str],
        deactivate_controllers: Iterable[str],
        completion_callback: Callable[[bool], None],
    ) -> None:
        """
        Send one asynchronous controller activation/deactivation request.

        Parameters:
            activate_controllers:
                Names of controllers that should become active.

            deactivate_controllers:
                Names of controllers that should become inactive.

            completion_callback:
                Function called with True when the switch succeeds or False
                when it fails.

        Returns:
            None.
        """
        activate_list = list(activate_controllers)
        deactivate_list = list(deactivate_controllers)

        if not self.switch_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT_SECONDS
        ):
            self.get_logger().error(
                "switch_controller service is unavailable"
            )
            self.operation_in_progress = False
            return

        request = SwitchController.Request()
        request.activate_controllers = activate_list
        request.deactivate_controllers = deactivate_list
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = SWITCH_TIMEOUT_SECONDS
        request.timeout.nanosec = 0

        future = self.switch_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.switch_request_finished(
                completed_future,
                activate_list,
                deactivate_list,
                completion_callback,
            )
        )

    def switch_request_finished(
        self,
        future,
        activate_controllers: list[str],
        deactivate_controllers: list[str],
        completion_callback: Callable[[bool], None],
    ) -> None:
        """
        Process a completed controller switch request.

        Parameters:
            future:
                Completed asynchronous SwitchController service future.

            activate_controllers:
                Controller names requested for activation.

            deactivate_controllers:
                Controller names requested for deactivation.

            completion_callback:
                Function that receives the final success state.

        Returns:
            None.
        """
        success = False

        try:
            response = future.result()

            if response is not None:
                success = bool(response.ok)

            if not success:
                self.get_logger().error(
                    "Controller switch failed: "
                    f"activate={activate_controllers}, "
                    f"deactivate={deactivate_controllers}"
                )

        except Exception as exception:
            self.get_logger().error(
                f"Controller switch exception: {exception}"
            )

        self.operation_in_progress = False
        completion_callback(success)

    def begin_safety_stop(self) -> None:
        """
        Start deactivating motion controllers for an active safety stop.

        The method publishes repeated zero commands before querying active
        controllers. It does nothing while another controller operation is
        running or after controllers have already been deactivated.

        Parameters:
            None.

        Returns:
            None.
        """
        if self.controllers_deactivated or self.operation_in_progress:
            return

        self.get_logger().error(
            "Safety stop active: stopping all robot motion"
        )

        self.publish_zero_repeatedly()
        self.request_active_motion_controllers()

    def deactivation_finished(self, success: bool) -> None:
        """
        Update supervisor state after a deactivation request.

        Parameters:
            success:
                True when all requested controller deactivations succeeded.

        Returns:
            None.
        """
        if success:
            self.controllers_deactivated = True
            self.get_logger().error("Motion controller deactivated")
        else:
            self.controllers_deactivated = False
            self.get_logger().error(
                "Motion controller deactivation failed; "
                "will retry while safety stop remains active"
            )

    def begin_safety_clear(self) -> None:
        """
        Reactivate controllers remembered from before the safety stop.

        Parameters:
            None.

        Returns:
            None.
        """
        if not self.controllers_deactivated or self.operation_in_progress:
            return

        if not self.controllers_to_reactivate:
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

    def reactivation_finished(self, success: bool) -> None:
        """
        Update supervisor state after a controller reactivation request.

        Parameters:
            success:
                True when all requested controller activations succeeded.

        Returns:
            None.
        """
        if success:
            self.controllers_to_reactivate.clear()
            self.controllers_deactivated = False
        else:
            self.get_logger().error(
                "Controller reactivation failed; will retry"
            )

    def update(self) -> None:
        """
        Execute one supervisor cycle.

        While safety is active, a zero velocity command is continuously
        published and motion controllers are deactivated. When safety clears,
        previously active controllers are restored.

        Parameters:
            None.

        Returns:
            None.
        """
        if self.get_safety_stop():
            self.publish_zero()
            self.begin_safety_stop()
        else:
            self.begin_safety_clear()


def main(args=None) -> None:
    """
    Initialize ROS 2, run the safety supervisor, and shut down safely.

    Parameters:
        args:
            Optional command-line arguments passed to rclpy.init(). When None,
            rclpy uses the process command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node: Optional[SafetySupervisor] = None

    try:
        node = SafetySupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_zero_repeatedly()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()