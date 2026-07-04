#!/usr/bin/env python3

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

COMMAND_TOPIC = "/servo_controller/commands"

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

PUBLISH_RATE = 50.0
MOVEMENT_TIMEOUT = 30.0

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


def clamp(value, low, high):
    return max(low, min(high, value))


class StartPositionServo(Node):

    def __init__(self):
        super().__init__("start_position_servo")

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.lock = threading.Lock()
        self.positions = {}

        self.get_logger().info(
            f"Publishing direct joint velocities to {COMMAND_TOPIC}"
        )

    def joint_state_callback(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.positions[name] = float(position)

    def get_positions(self):
        with self.lock:
            return dict(self.positions)

    def wait_for_joint_states(self, timeout=10.0):
        start_time = time.monotonic()

        self.get_logger().info("Waiting for joint states...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            positions = self.get_positions()

            if all(joint in positions for joint in JOINT_NAMES):
                self.get_logger().info("All joint states received")
                return True

            if time.monotonic() - start_time > timeout:
                self.get_logger().error(
                    "Timed out waiting for joint states"
                )
                return False

        return False

    def publish_command(self, commands):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in commands]
        self.command_pub.publish(msg)

    def stop(self, count=10):
        for _ in range(count):
            self.publish_command([0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.02)

    def compute_velocity(self, joint, current, target):
        error = target - current

        if abs(error) <= TOLERANCE[joint]:
            return 0.0

        velocity = KP[joint] * error

        velocity = clamp(
            velocity,
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

    def go_to_target(self, target, target_name):
        self.get_logger().warn(
            f"Moving to {target_name}"
        )

        if not self.wait_for_joint_states():
            self.stop()
            return False

        movement_start = time.monotonic()
        last_print = 0.0
        period = 1.0 / PUBLISH_RATE

        while rclpy.ok():
            loop_start = time.monotonic()

            rclpy.spin_once(self, timeout_sec=0.001)

            if loop_start - movement_start > MOVEMENT_TIMEOUT:
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
                desired = target[joint]

                velocity = self.compute_velocity(
                    joint,
                    current,
                    desired,
                )

                commands.append(velocity)

                if abs(desired - current) > TOLERANCE[joint]:
                    all_reached = False

            if all_reached:
                self.stop()
                self.print_positions(target)

                self.get_logger().warn(
                    f"Reached {target_name}"
                )
                return True

            self.publish_command(commands)

            now = time.monotonic()

            if now - last_print >= 0.5:
                last_print = now
                self.print_positions(target)

            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed

            if sleep_time > 0.0:
                time.sleep(sleep_time)

        self.stop()
        return False

    def print_positions(self, target):
        positions = self.get_positions()

        print("")
        print("Current joint positions:")

        for joint in JOINT_NAMES:
            current = positions.get(joint)

            if current is None:
                print(f"  {joint}: unknown")
                continue

            error = target[joint] - current

            print(
                f"  {joint}: {current:+.4f} rad   "
                f"target {target[joint]:+.4f}   "
                f"error {error:+.4f}"
            )


def parse_arguments():
    parser = argparse.ArgumentParser(
        add_help=False,
        description="Move robot using the Servo velocity controller",
    )

    group = parser.add_mutually_exclusive_group(required=True)

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

    parser.add_argument(
        "--help",
        action="help",
        help="Show this help message",
    )

    return parser.parse_args()


def main(args=None):
    cli_args = parse_arguments()

    rclpy.init(args=args)

    node = StartPositionServo()
    success = False

    try:
        if cli_args.start:
            success = node.go_to_target(
                START_POSITION,
                "Servo start position",
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
