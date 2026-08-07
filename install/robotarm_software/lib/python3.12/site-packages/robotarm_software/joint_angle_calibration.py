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

JOINT_ALIASES = {
    "base": "base_joint",
    "b": "base_joint",
    "base_joint": "base_joint",

    "shoulder": "shoulder_joint",
    "s": "shoulder_joint",
    "shoulder_joint": "shoulder_joint",

    "elbow": "elbow_joint",
    "e": "elbow_joint",
    "elbow_joint": "elbow_joint",
}

COMMAND_TOPIC = "/velocity_controller/commands"

PUBLISH_RATE = 50.0

# Für deinen Roboter: go_to_start.py nutzt 0.08 zuverlässig.
DEFAULT_SPEED_RAD_S = 0.08

# Sicherheit: maximal erlaubte Einzelbewegung pro Test.
MAX_MOVE_DEG = 90.0

# Stoppt, wenn Ziel näher als diese Toleranz erreicht ist.
TOLERANCE_DEG = 1.0

# Harte Joint Limits wie in deinen anderen Scripts.
JOINT_LIMITS = {
    "base_joint": (-3.0, 3.0),
    "shoulder_joint": (-0.52359878, 1.39626340),
    "elbow_joint": (-0.69813170, 2.44346095),
}

JOINT_LIMIT_MARGIN = 0.04


def clamp(value, low, high):
    return max(low, min(high, value))


def deg_to_rad(deg):
    return deg * math.pi / 180.0


def rad_to_deg(rad):
    return rad * 180.0 / math.pi


class JointAngleCalibration(Node):

    def __init__(self):
        super().__init__("joint_angle_calibration")

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
        self.velocities = {}

        self.printed_order = False

        self.get_logger().info("Joint angle calibration ready")
        self.get_logger().info("Publishes direct joint velocities to /velocity_controller/commands")
        self.get_logger().warn("Move only one joint at a time. Keep hand near emergency stop.")

    def joint_state_callback(self, msg):
        with self.lock:
            if not self.printed_order:
                self.printed_order = True
                self.get_logger().warn(
                    "Received /joint_states order: " + ", ".join(msg.name)
                )
                self.get_logger().warn(
                    "This script reads joint states BY NAME, not by index."
                )

            for name, pos in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.positions[name] = float(pos)

            for name, vel in zip(msg.name, msg.velocity):
                if name in JOINT_NAMES:
                    self.velocities[name] = float(vel)

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

    def publish_command_for_joint(self, joint, velocity):
        cmd = [0.0, 0.0, 0.0]

        index = JOINT_NAMES.index(joint)
        cmd[index] = velocity

        msg = Float64MultiArray()
        msg.data = cmd
        self.pub.publish(msg)

    def stop(self, count=10):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]

        for _ in range(count):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.02)

    def print_current_positions(self):
        positions = self.get_positions()

        print("")
        print("Current joint positions:")

        for joint in JOINT_NAMES:
            if joint in positions:
                print(
                    f"  {joint}: "
                    f"{positions[joint]:+.4f} rad  "
                    f"{rad_to_deg(positions[joint]):+.2f} deg"
                )
            else:
                print(f"  {joint}: unknown")

    def move_joint_relative_degrees(self, joint, delta_deg, speed_rad_s):
        if abs(delta_deg) > MAX_MOVE_DEG:
            print("")
            print(f"Refusing: {delta_deg:.1f} deg is larger than MAX_MOVE_DEG={MAX_MOVE_DEG:.1f}")
            print("Use smaller calibration moves.")
            return False

        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states received")
            self.stop()
            return False

        positions = self.get_positions()
        start_pos = positions[joint]

        delta_rad = deg_to_rad(delta_deg)
        target_pos = start_pos + delta_rad

        lower, upper = JOINT_LIMITS[joint]
        safe_lower = lower + JOINT_LIMIT_MARGIN
        safe_upper = upper - JOINT_LIMIT_MARGIN

        if target_pos < safe_lower or target_pos > safe_upper:
            print("")
            print("Target would exceed safe joint limit.")
            print(f"  joint: {joint}")
            print(f"  current: {start_pos:+.4f} rad / {rad_to_deg(start_pos):+.2f} deg")
            print(f"  requested delta: {delta_rad:+.4f} rad / {delta_deg:+.2f} deg")
            print(f"  target: {target_pos:+.4f} rad / {rad_to_deg(target_pos):+.2f} deg")
            print(f"  safe range: {safe_lower:+.4f} to {safe_upper:+.4f} rad")
            return False

        direction = 1.0 if delta_rad > 0.0 else -1.0
        command_velocity = direction * abs(speed_rad_s)

        tolerance_rad = deg_to_rad(TOLERANCE_DEG)

        print("")
        print("Calibration move")
        print(f"  joint: {joint}")
        print(f"  start:  {start_pos:+.4f} rad / {rad_to_deg(start_pos):+.2f} deg")
        print(f"  target: {target_pos:+.4f} rad / {rad_to_deg(target_pos):+.2f} deg")
        print(f"  delta:  {delta_rad:+.4f} rad / {delta_deg:+.2f} deg")
        print(f"  command velocity: {command_velocity:+.4f} rad/s")
        print("")
        print("After it stops, measure the REAL physical angle change and compare.")
        print("Press Ctrl+C to abort if needed.")
        print("")

        dt = 1.0 / PUBLISH_RATE
        last_print = 0.0

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)

                positions = self.get_positions()

                if joint not in positions:
                    self.get_logger().warn("Missing joint state")
                    self.stop()
                    return False

                current = positions[joint]
                remaining = target_pos - current

                reached = abs(remaining) <= tolerance_rad

                # Also stop if we passed the target.
                if direction > 0.0 and current >= target_pos:
                    reached = True

                if direction < 0.0 and current <= target_pos:
                    reached = True

                if reached:
                    self.stop()
                    final_positions = self.get_positions()
                    final_pos = final_positions[joint]
                    measured_delta = final_pos - start_pos

                    print("")
                    print("Reached encoder target.")
                    print(f"  joint: {joint}")
                    print(f"  start encoder angle: {start_pos:+.4f} rad / {rad_to_deg(start_pos):+.2f} deg")
                    print(f"  final encoder angle: {final_pos:+.4f} rad / {rad_to_deg(final_pos):+.2f} deg")
                    print(f"  encoder delta:       {measured_delta:+.4f} rad / {rad_to_deg(measured_delta):+.2f} deg")
                    print("")
                    print("Now measure the REAL physical angle change.")
                    print("")
                    print("If real angle != encoder delta, calculate correction:")
                    print("")
                    print("  new_ticks_per_joint_rev = old_ticks_per_joint_rev * (real_deg / encoder_deg)")
                    print("")
                    print("Example:")
                    print("  Script says encoder moved 45 deg")
                    print("  Real measurement says it moved 60 deg")
                    print("  new_ticks = old_ticks * (60 / 45)")
                    print("")
                    return True

                self.publish_command_for_joint(joint, command_velocity)

                now = time.monotonic()

                if now - last_print > 0.5:
                    last_print = now
                    print(
                        f"  current {joint}: "
                        f"{current:+.4f} rad / {rad_to_deg(current):+.2f} deg | "
                        f"remaining {remaining:+.4f} rad / {rad_to_deg(remaining):+.2f} deg"
                    )

                time.sleep(dt)

        except KeyboardInterrupt:
            print("")
            print("Aborted by user.")
            self.stop()
            return False

        self.stop()
        return False


def parse_joint(text):
    key = text.strip().lower()

    if key not in JOINT_ALIASES:
        return None

    return JOINT_ALIASES[key]


def main(args=None):
    rclpy.init(args=args)

    node = JointAngleCalibration()

    try:
        if not node.wait_for_joint_states():
            print("No joint states received.")
            node.stop()
            return

        while rclpy.ok():
            print("")
            print("Choose:")
            print("  p = print current joint positions")
            print("  q = quit")
            print("  or enter joint and degrees")
            print("")
            print("Examples:")
            print("  elbow 30")
            print("  elbow -30")
            print("  shoulder 20")
            print("  base -45")
            print("")

            text = input("Command: ").strip()

            if text.lower() == "q":
                node.stop()
                break

            if text.lower() == "p":
                node.print_current_positions()
                continue

            parts = text.split()

            if len(parts) != 2:
                print("Need: <joint> <degrees>")
                print("Example: elbow 30")
                continue

            joint = parse_joint(parts[0])

            if joint is None:
                print("Unknown joint. Use: base, shoulder, elbow")
                continue

            try:
                delta_deg = float(parts[1])
            except ValueError:
                print("Degrees must be a number.")
                continue

            if abs(delta_deg) < 0.1:
                print("Move is too small.")
                continue

            node.move_joint_relative_degrees(
                joint=joint,
                delta_deg=delta_deg,
                speed_rad_s=DEFAULT_SPEED_RAD_S,
            )

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
