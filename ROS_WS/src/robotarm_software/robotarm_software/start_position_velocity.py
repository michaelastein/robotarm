#!/usr/bin/env python3
"""
ROS 2 command-line node for moving the robot arm with direct joint velocities.

The node publishes velocity commands to the servo controller until every joint
reaches a predefined target within its configured tolerance.

Command-line options:
    --start or -s:
        Move the arm to START_POSITION.

    --zero or -z:
        Move the arm to HOME_POSITION.

Exactly one option must be selected.

Joint configuration:
    JOINT_NAMES:
        Ordered joint list used for command messages and state validation.

    START_POSITION:
        Predefined start position in radians, indexed by joint name.

    HOME_POSITION:
        Zero or home position in radians, indexed by joint name.

Control parameters:
    PUBLISH_RATE_HZ:
        Frequency at which velocity commands are sent.

    MOVEMENT_TIMEOUT_SECONDS:
        Maximum duration allowed for one complete movement.

    JOINT_STATE_TIMEOUT_SECONDS:
        Maximum time to wait for all required joint states before movement.

    KP:
        Proportional gain for each joint. Commanded velocity is initially
        calculated as gain multiplied by position error.

    MAX_VEL:
        Maximum absolute velocity in radians per second for each joint.

    MIN_VEL:
        Minimum effective absolute velocity in radians per second. When a joint
        is outside its tolerance but the proportional command is smaller than
        this value, the command is raised to the configured minimum while
        preserving direction.

    TOLERANCE:
        Maximum permitted absolute position error in radians for considering a
        joint to have reached its target.

Joint protection:
    JOINT_LIMITS:
        Lower and upper permitted joint angles in radians.

    JOINT_LIMIT_MARGIN:
        Safety margin inside each joint limit. Commands that would move farther
        beyond a nearby limit are blocked.

Stopping behavior:
    ZERO_COMMAND_COUNT:
        Number of zero-velocity commands published when movement completes,
        fails, is interrupted, or the process shuts down.

    ZERO_COMMAND_DELAY_SECONDS:
        Delay between repeated zero commands.

Exit status:
    0:
        The requested target was reached successfully.

    1:
        Joint states were unavailable, movement timed out, a required state was
        lost, ROS stopped, or the movement was interrupted.
"""

import argparse
import math
import threading
import time
from collections.abc import Mapping, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

JOINT_STATE_TOPIC = "/joint_states"
COMMAND_TOPIC = "/velocity_controller/commands"

START_POSITION = {
    "base_joint": 0.3,
    "shoulder_joint": 0.43,
    "elbow_joint": 1.1,
}

HOME_POSITION = {
    "base_joint": 0.0,
    "shoulder_joint": 0.0,
    "elbow_joint": 0.0,
}

PUBLISH_RATE_HZ = 50.0
MOVEMENT_TIMEOUT_SECONDS = 30.0
JOINT_STATE_TIMEOUT_SECONDS = 10.0

KP = {
    "base_joint": 0.8,
    "shoulder_joint": 0.8,
    "elbow_joint": 0.8,
}

MAX_VEL = {
    "base_joint": 0.08,
    "shoulder_joint": 0.08,
    "elbow_joint": 0.08,
}

MIN_VEL = {
    "base_joint": 0.02,
    "shoulder_joint": 0.08,
    "elbow_joint": 0.08,
}

TOLERANCE = {
    "base_joint": 0.035,
    "shoulder_joint": 0.035,
    "elbow_joint": 0.035,
}

JOINT_LIMITS = {
    "base_joint": (-3.0, 3.0),
    "shoulder_joint": (-0.52359878, 1.39626340),
    "elbow_joint": (-0.69813170, 2.44346095),
}

JOINT_LIMIT_MARGIN = 0.04

ZERO_COMMAND_COUNT = 10
ZERO_COMMAND_DELAY_SECONDS = 0.02


def clamp(value: float, low: float, high: float) -> float:
    """
    Restrict a numeric value to an inclusive interval.

    Parameters:
        value:
            Number to restrict.

        low:
            Minimum permitted value.

        high:
            Maximum permitted value.

    Returns:
        value when it is already inside the interval, otherwise the nearest
        boundary.
    """
    return max(low, min(high, value))


class StartPositionVelocity(Node):
    """Move predefined joint targets using direct velocity commands."""

    def __init__(self) -> None:
        """
        Initialize the velocity publisher and joint-state subscription.

        Parameters:
            None.

        Returns:
            None.
        """
        super().__init__("start_position_velocity")

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        self.joint_sub = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.lock = threading.Lock()
        self.positions: dict[str, float] = {}

        self.get_logger().info(
            f"Publishing direct joint velocities to {COMMAND_TOPIC}"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        """
        Store the latest position for each controlled joint.

        Parameters:
            msg:
                JointState message. Joint values are matched by name rather
                than by their index in the incoming message.

        Returns:
            None.
        """
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.positions[name] = float(position)

    def get_positions(self) -> dict[str, float]:
        """
        Return a thread-safe snapshot of the latest joint positions.

        Parameters:
            None.

        Returns:
            Dictionary mapping each received controlled joint name to its latest
            position in radians.
        """
        with self.lock:
            return dict(self.positions)

    def wait_for_joint_states(
        self,
        timeout: float = JOINT_STATE_TIMEOUT_SECONDS,
    ) -> bool:
        """
        Wait until positions for every controlled joint have been received.

        Parameters:
            timeout:
                Maximum wait duration in seconds.

        Returns:
            True when all joints are available before the timeout.
            False when the timeout expires or ROS shuts down.
        """
        start_time = time.monotonic()

        self.get_logger().info("Waiting for joint states...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            positions = self.get_positions()

            if all(joint in positions for joint in JOINT_NAMES):
                return True

            if time.monotonic() - start_time > timeout:
                self.get_logger().error(
                    "Timed out waiting for joint states"
                )
                return False

        return False

    def publish_command(self, commands: Sequence[float]) -> None:
        """
        Publish one ordered joint-velocity command.

        Parameters:
            commands:
                Velocity values in radians per second. The order must match
                JOINT_NAMES.

        Returns:
            None.
        """
        msg = Float64MultiArray()
        msg.data = [float(value) for value in commands]
        self.command_pub.publish(msg)

    def stop(self, count: int = ZERO_COMMAND_COUNT) -> None:
        """
        Publish repeated zero commands to stop all controlled joints.

        Parameters:
            count:
                Number of zero command messages to publish.

        Returns:
            None.
        """
        zero_command = [0.0] * len(JOINT_NAMES)

        for _ in range(count):
            self.publish_command(zero_command)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(ZERO_COMMAND_DELAY_SECONDS)

    def compute_velocity(
        self,
        joint: str,
        current: float,
        target: float,
    ) -> float:
        """
        Compute a bounded proportional velocity for one joint.

        Parameters:
            joint:
                Joint name used to select gains, limits, and tolerances.

            current:
                Current joint angle in radians.

            target:
                Desired joint angle in radians.

        Returns:
            Signed velocity command in radians per second.

            Zero is returned when the joint is within tolerance or when motion
            would continue farther beyond a nearby joint limit.
        """
        error = target - current

        if abs(error) <= TOLERANCE[joint]:
            return 0.0

        velocity = clamp(
            KP[joint] * error,
            -MAX_VEL[joint],
            MAX_VEL[joint],
        )

        if abs(velocity) < MIN_VEL[joint]:
            velocity = math.copysign(
                MIN_VEL[joint],
                velocity,
            )

        lower, upper = JOINT_LIMITS[joint]

        if (
            current <= lower + JOINT_LIMIT_MARGIN
            and velocity < 0.0
        ):
            return 0.0

        if (
            current >= upper - JOINT_LIMIT_MARGIN
            and velocity > 0.0
        ):
            return 0.0

        return velocity

    def go_to_target(
        self,
        target: Mapping[str, float],
        target_name: str,
    ) -> bool:
        """
        Drive all controlled joints toward one predefined target.

        Parameters:
            target:
                Mapping from each name in JOINT_NAMES to its target position in
                radians.

            target_name:
                Human-readable name used in status and error messages.

        Returns:
            True when every joint reaches its configured tolerance.

            False when joint states are unavailable, become incomplete,
            movement times out, or ROS stops.
        """
        missing_targets = [
            joint for joint in JOINT_NAMES
            if joint not in target
        ]

        if missing_targets:
            self.get_logger().error(
                "Target is missing joint(s): "
                + ", ".join(missing_targets)
            )
            return False

        self.get_logger().warn(
            f"Moving to {target_name}"
        )

        if not self.wait_for_joint_states():
            self.stop()
            return False

        movement_start = time.monotonic()
        period = 1.0 / PUBLISH_RATE_HZ

        while rclpy.ok():
            loop_start = time.monotonic()

            rclpy.spin_once(self, timeout_sec=0.001)

            if (
                loop_start - movement_start
                > MOVEMENT_TIMEOUT_SECONDS
            ):
                self.get_logger().error(
                    f"Movement to {target_name} timed out"
                )
                self.stop()
                return False

            positions = self.get_positions()

            if not all(joint in positions for joint in JOINT_NAMES):
                self.get_logger().error(
                    "Missing joint states during movement"
                )
                self.stop()
                return False

            commands = []
            all_reached = True

            for joint in JOINT_NAMES:
                current = positions[joint]
                desired = float(target[joint])

                commands.append(
                    self.compute_velocity(
                        joint,
                        current,
                        desired,
                    )
                )

                if abs(desired - current) > TOLERANCE[joint]:
                    all_reached = False

            if all_reached:
                self.stop()
                self.get_logger().info(
                    f"Reached {target_name}"
                )
                return True

            self.publish_command(commands)

            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed

            if sleep_time > 0.0:
                time.sleep(sleep_time)

        self.stop()
        return False


def parse_arguments() -> argparse.Namespace:
    """
    Parse the required start-or-zero command-line selection.

    Parameters:
        None. Arguments are read from the process command line.

    Returns:
        argparse.Namespace containing:
            start:
                True when --start or -s was selected.

            zero:
                True when --zero or -z was selected.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Move the robot using the direct velocity control "
            "velocity controller"
        ),
    )

    group = parser.add_mutually_exclusive_group(
        required=True
    )

    group.add_argument(
        "-s",
        "--start",
        action="store_true",
        help="Move to the predefined start position",
    )

    group.add_argument(
        "-z",
        "--zero",
        action="store_true",
        help="Move to the zero/home position",
    )

    return parser.parse_args()


def main(args=None) -> None:
    """
    Parse the target, initialize ROS 2, execute movement, and exit.

    Parameters:
        args:
            Optional ROS-specific command-line arguments passed to rclpy.init().
            The start/zero selection is parsed separately by argparse.

    Returns:
        None. The process exits with status 0 on success and 1 on failure.
    """
    cli_args = parse_arguments()

    rclpy.init(args=args)
    node = StartPositionVelocity()
    success = False

    try:
        if cli_args.start:
            success = node.go_to_target(
                START_POSITION,
                "Velocity start position",
            )
        elif cli_args.zero:
            success = node.go_to_target(
                HOME_POSITION,
                "zero position",
            )

    except KeyboardInterrupt:
        node.get_logger().warn("Movement interrupted")

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()