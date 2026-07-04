#!/usr/bin/env python3

import math
import time
import threading

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

ZERO_POSITION = {
    "base_joint": 0.0,
    "shoulder_joint": 0.0,
    "elbow_joint": 0.0,
}

PUBLISH_RATE = 50.0

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


class GoToJointTarget(Node):

    def __init__(self):
        super().__init__("go_to_joint_target")

        self.pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.lock = threading.Lock()
        self.positions = {}

        self.get_logger().info("go_to_start ready")
        self.get_logger().info("Publishes direct joint velocities to /servo_controller/commands")

    def joint_state_callback(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.positions[name] = float(pos)

    def get_positions(self):
        with self.lock:
            return dict(self.positions)

    def wait_for_joint_states(self, timeout=5.0):
        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            positions = self.get_positions()

            if all(joint in positions for joint in JOINT_NAMES):
                return True

            if time.monotonic() - start > timeout:
                return False

        return False

    def publish_command(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self.pub.publish(msg)

    def stop(self, count=10):
        for _ in range(count):
            self.publish_command([0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.02)

    def compute_velocity(self, joint, current, target):
        error = target - current

        if abs(error) <= TOLERANCE[joint]:
            return 0.0

        vel = KP[joint] * error
        vel = clamp(
            vel,
            -MAX_VEL[joint],
            MAX_VEL[joint],
        )

        if abs(vel) < MIN_VEL[joint]:
            vel = math.copysign(MIN_VEL[joint], vel)

        lower, upper = JOINT_LIMITS[joint]

        if current <= lower + JOINT_LIMIT_MARGIN and vel < 0.0:
            return 0.0

        if current >= upper - JOINT_LIMIT_MARGIN and vel > 0.0:
            return 0.0

        return vel

    def go_to_target(self, target, name):
        self.get_logger().warn(f"Moving to {name}")

        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states received")
            self.stop()
            return False

        dt = 1.0 / PUBLISH_RATE
        last_print = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

            positions = self.get_positions()

            if not all(joint in positions for joint in JOINT_NAMES):
                self.get_logger().warn("Missing joint state")
                self.stop()
                return False

            commands = []
            all_reached = True

            for joint in JOINT_NAMES:
                current = positions[joint]
                vel = self.compute_velocity(
                    joint,
                    current,
                    target[joint],
                )

                commands.append(vel)

                if abs(target[joint] - current) > TOLERANCE[joint]:
                    all_reached = False

            if all_reached:
                self.stop()
                self.print_positions(target)
                self.get_logger().warn(f"Reached {name}")
                return True

            self.publish_command(commands)

            now = time.monotonic()
            if now - last_print > 0.5:
                last_print = now
                self.print_positions(target)

            time.sleep(dt)

        self.stop()
        return False

    def print_positions(self, target):
        positions = self.get_positions()

        print("")
        print("Current joint positions:")

        for joint in JOINT_NAMES:
            current = positions.get(joint, None)

            if current is None:
                print(f"  {joint}: unknown")
            else:
                error = target[joint] - current
                print(
                    f"  {joint}: {current:+.4f} rad   "
                    f"target {target[joint]:+.4f}   "
                    f"error {error:+.4f}"
                )


def main(args=None):
    rclpy.init(args=args)

    node = GoToJointTarget()

    try:
        print("")
        print("Choose target:")
        print("  g = go to start position")
        print("  h = go to zero position")
        print("  p = print current position only")
        print("  q = quit")
        print("")

        choice = input("Command [g/h/p/q]: ").strip().lower()

        if choice == "g":
            node.go_to_target(START_POSITION, "start position")
        elif choice == "h":
            node.go_to_target(ZERO_POSITION, "zero position")
        elif choice == "p":
            if node.wait_for_joint_states():
                node.print_positions(START_POSITION)
            else:
                print("No joint states received")
        else:
            node.stop()

    except KeyboardInterrupt:
        pass

    node.stop()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
