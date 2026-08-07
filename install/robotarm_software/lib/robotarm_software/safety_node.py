#!/usr/bin/env python3
"""
ROS 2 safety node for a GPIO emergency-stop and resume-button circuit.

The node continuously reads two GPIO inputs and publishes a latched safety-stop
state. Once a safety fault occurs, the stop remains active until all faults are
clear and the resume button is pressed.

Published topics:
    SAFETY_STOP_TOPIC:
        std_msgs/msg/Bool.

        True:
            Motion must stop because a safety condition is active or latched.

        False:
            No safety stop is currently active.

    SAFETY_REASON_TOPIC:
        std_msgs/msg/String containing one or more comma-separated reason codes.

        Possible values:
            "ok"
                No safety fault is active.

            "emergency_button_or_cable"
                The emergency input is HIGH. With the fail-safe wiring described
                below, this represents a pressed/open emergency switch or a
                disconnected cable.

            "distance_too_small"
                Reserved for future distance-sensor integration.

            "latched_stop_waiting_for_resume"
                The original fault has cleared, but the stop remains latched
                until the resume button is pressed.

            "resume_ignored_fault_still_active"
                Resume was pressed while a safety fault was still active.

GPIO configuration:
    GPIO_CHIP:
        Linux GPIO chip number passed to lgpio.gpiochip_open(). A value of 4
        corresponds to /dev/gpiochip4.

    EMERGENCY_GPIO:
        GPIO line used for the fail-safe emergency-stop circuit.

        The input uses an internal pull-up:
            LOW:
                Safe state. The switch/cable connects the GPIO to ground.

            HIGH:
                Stop state. This can indicate a pressed/open switch, broken
                wire, unplugged connector, or loss of the ground connection.

    RESUME_GPIO:
        GPIO line used for the manual resume button.

        The input uses an internal pull-up:
            LOW:
                Button pressed.

            HIGH:
                Button released.

Timing:
    UPDATE_PERIOD_SECONDS:
        Time between safety checks and topic publications. A value of 0.02
        seconds corresponds to 50 Hz.

Distance sensor:
    USE_DISTANCE_SENSOR:
        Enables the distance-stop check when future sensor integration is added.

    MIN_DISTANCE_M:
        Minimum permitted distance in metres. The current placeholder distance
        function does not yet read a real sensor.
"""

from typing import Optional

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


GPIO_CHIP = 4

EMERGENCY_GPIO = 6
RESUME_GPIO = 17

SAFETY_STOP_TOPIC = "/robotarm/safety_stop"
SAFETY_REASON_TOPIC = "/robotarm/safety_reason"

UPDATE_PERIOD_SECONDS = 0.02

USE_DISTANCE_SENSOR = False
MIN_DISTANCE_M = 0.15


class SafetyNode(Node):
    """Monitor safety inputs and publish a latched robot safety-stop state."""

    def __init__(self) -> None:
        """
        Initialize ROS publishers, GPIO inputs, and the safety update timer.

        Parameters:
            None.

        Returns:
            None.

        Raises:
            RuntimeError:
                May be raised by lgpio if the configured GPIO chip cannot be
                opened or either GPIO line cannot be claimed.
        """
        super().__init__("safety_node")

        self.safety_pub = self.create_publisher(
            Bool,
            SAFETY_STOP_TOPIC,
            10,
        )

        self.reason_pub = self.create_publisher(
            String,
            SAFETY_REASON_TOPIC,
            10,
        )

        self.gpio_handle = lgpio.gpiochip_open(GPIO_CHIP)

        lgpio.gpio_claim_input(
            self.gpio_handle,
            EMERGENCY_GPIO,
            lgpio.SET_PULL_UP,
        )

        lgpio.gpio_claim_input(
            self.gpio_handle,
            RESUME_GPIO,
            lgpio.SET_PULL_UP,
        )

        self.safety_latched = False
        self.last_safety_stop: Optional[bool] = None
        self.last_reason = ""

        self.timer = self.create_timer(
            UPDATE_PERIOD_SECONDS,
            self.update,
        )

        self.get_logger().info("Safety node started")

    def read_emergency_stop(self) -> bool:
        """
        Read the fail-safe emergency-stop input.

        Parameters:
            None.

        Returns:
            True when the emergency input is HIGH and a stop must be triggered.
            False when the input is LOW and the emergency circuit is safe.
        """
        value = lgpio.gpio_read(
            self.gpio_handle,
            EMERGENCY_GPIO,
        )
        return value == 1

    def read_resume_button(self) -> bool:
        """
        Read the active-low resume button.

        Parameters:
            None.

        Returns:
            True when the GPIO reading is HIGH, matching the behavior used by
            the original working implementation.

            False when the GPIO reading is LOW.
        """
        value = lgpio.gpio_read(
            self.gpio_handle,
            RESUME_GPIO,
        )
        return value == 1

    def read_distance_sensor_stop(self) -> bool:
        """
        Check whether a distance sensor requires a safety stop.

        Parameters:
            None.

        Returns:
            False in the current implementation because no distance sensor has
            been integrated yet.

        Notes:
            Future code should return True when a measured distance is below
            MIN_DISTANCE_M or when a sensor failure must be treated as unsafe.
        """
        if not USE_DISTANCE_SENSOR:
            return False

        return False

    def update(self) -> None:
        """
        Read all safety inputs, update the latch, and publish the safety state.

        A detected fault immediately sets the latch. The latch clears only when
        every fault is inactive and the resume button is pressed.

        Parameters:
            None.

        Returns:
            None.
        """
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
                reasons.clear()
            else:
                reasons.append("resume_ignored_fault_still_active")

        if self.safety_latched and not reasons:
            reasons.append("latched_stop_waiting_for_resume")

        reason_text = ",".join(reasons) if reasons else "ok"

        safety_message = Bool()
        safety_message.data = self.safety_latched
        self.safety_pub.publish(safety_message)

        reason_message = String()
        reason_message.data = reason_text
        self.reason_pub.publish(reason_message)

        if (
            self.safety_latched != self.last_safety_stop
            or reason_text != self.last_reason
        ):
            if self.safety_latched:
                self.get_logger().error(
                    f"SAFETY STOP ACTIVE: {reason_text}"
                )
            else:
                self.get_logger().info("Safety clear")

            self.last_safety_stop = self.safety_latched
            self.last_reason = reason_text

    def destroy_node(self) -> None:
        """
        Release claimed GPIO lines, close the GPIO chip, and destroy the node.

        Parameters:
            None.

        Returns:
            None.
        """
        for pin in (EMERGENCY_GPIO, RESUME_GPIO):
            try:
                lgpio.gpio_free(self.gpio_handle, pin)
            except Exception:
                pass

        try:
            lgpio.gpiochip_close(self.gpio_handle)
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    """
    Initialize ROS 2, run the safety node, and shut it down safely.

    Parameters:
        args:
            Optional command-line arguments passed to rclpy.init(). When None,
            rclpy uses the process command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = SafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()