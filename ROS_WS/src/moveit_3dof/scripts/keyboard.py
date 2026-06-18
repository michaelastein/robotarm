#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.signals import SignalHandlerOptions

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = RuntimeError

from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ServoCommandType


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

START_POSITION = {
    "base_joint": 0.0,
    "shoulder_joint": 0.8,
    "elbow_joint": 1.0,
}

ZERO_POSITION = {
    "base_joint": 0.0,
    "shoulder_joint": 0.0,
    "elbow_joint": 0.0,
}

VEL = 0.3

AUTO_KP = 0.8
AUTO_MAX_VEL = 0.12
AUTO_MIN_VEL = 0.025
AUTO_TOLERANCE = 0.025

PUBLISH_RATE = 50.0


def clamp(value, low, high):
    return max(low, min(high, value))


def format_target(target):
    return "\n".join(
        f"{joint:<14} = {target[joint]:.4f}"
        for joint in JOINT_NAMES
    )


def get_help_text():
    return f"""
Keyboard MoveIt Servo Joint Jog
--------------------------------
Manual:
q / a  : base_joint - / +
w / s  : shoulder_joint + / -
e / d  : elbow_joint + / -

Automatic:
g      : go to start position
h      : go to 0 0 0

Other:
space  : stop
p      : print current position
x      : quit

Start target:
{format_target(START_POSITION)}

Zero target:
{format_target(ZERO_POSITION)}
"""


class KeyboardJointServo(Node):
    def __init__(self):
        super().__init__("keyboard_joint_servo")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(
            JointJog,
            "/servo_node/delta_joint_cmds",
            qos,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )

        self.current_positions = {}

        self.active_velocities = {
            "base_joint": 0.0,
            "shoulder_joint": 0.0,
            "elbow_joint": 0.0,
        }

        self.auto_target = None
        self.auto_name = None

        self.switch_to_joint_jog()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_command,
        )

        print(get_help_text())

    def switch_to_joint_jog(self):
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /servo_node/switch_command_type..."
            )

        if not rclpy.ok():
            return

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info(
                "MoveIt Servo switched to JOINT_JOG mode"
            )
        else:
            self.get_logger().error(
                "Failed to switch MoveIt Servo to JOINT_JOG mode"
            )

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos

    def stop(self):
        self.auto_target = None
        self.auto_name = None

        for joint in JOINT_NAMES:
            self.active_velocities[joint] = 0.0

    def start_auto_move(self, target, name):
        if not all(joint in self.current_positions for joint in JOINT_NAMES):
            self.get_logger().warn(
                "Cannot start auto move yet: waiting for all joint states"
            )
            return

        self.auto_target = target
        self.auto_name = name

        self.get_logger().warn(
            f"Automatic move started: {name}"
        )

    def update_auto_velocities(self):
        if self.auto_target is None:
            return False

        if not all(joint in self.current_positions for joint in JOINT_NAMES):
            self.stop()
            self.get_logger().warn(
                "Auto move stopped: missing joint states"
            )
            return False

        all_reached = True

        for joint in JOINT_NAMES:
            error = self.auto_target[joint] - self.current_positions[joint]

            if abs(error) <= AUTO_TOLERANCE:
                self.active_velocities[joint] = 0.0
                continue

            all_reached = False

            vel = AUTO_KP * error
            vel = clamp(
                vel,
                -AUTO_MAX_VEL,
                AUTO_MAX_VEL,
            )

            if abs(vel) < AUTO_MIN_VEL:
                vel = AUTO_MIN_VEL if vel > 0.0 else -AUTO_MIN_VEL

            self.active_velocities[joint] = vel

        if all_reached:
            self.get_logger().warn(
                f"Automatic move reached target: {self.auto_name}"
            )
            self.stop()
            self.print_positions()
            return False

        return True

    def set_key_command(self, key):
        self.auto_target = None
        self.auto_name = None

        for joint in JOINT_NAMES:
            self.active_velocities[joint] = 0.0

        if key == "q":
            self.active_velocities["base_joint"] = -0.3 *VEL
        elif key == "a":
            self.active_velocities["base_joint"] = 0.3 *VEL

        elif key == "w":
            self.active_velocities["shoulder_joint"] = -VEL
        elif key == "s":
            self.active_velocities["shoulder_joint"] = VEL

        elif key == "e":
            self.active_velocities["elbow_joint"] = -VEL
        elif key == "d":
            self.active_velocities["elbow_joint"] = VEL

        elif key == " ":
            self.stop()

    def publish_command(self):
        if not rclpy.ok():
            return

        if self.auto_target is not None:
            self.update_auto_velocities()

        msg = JointJog()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.joint_names = JOINT_NAMES
        msg.velocities = [
            self.active_velocities["base_joint"],
            self.active_velocities["shoulder_joint"],
            self.active_velocities["elbow_joint"],
        ]

        msg.duration = 0.1

        try:
            self.pub.publish(msg)
        except RCLError:
            pass

    def publish_stop_commands(self, count=10):
        self.stop()

        for _ in range(count):
            if not rclpy.ok():
                return
            self.publish_command()

    def print_positions(self):
        print("\nCurrent joint positions:")

        for joint in JOINT_NAMES:
            pos = self.current_positions.get(joint, None)
            start_target = START_POSITION[joint]
            zero_target = ZERO_POSITION[joint]

            if pos is None:
                print(f"  {joint}: unknown")
            else:
                print(
                    f"  {joint}: {pos:+.3f} rad   "
                    f"start error {start_target - pos:+.3f}   "
                    f"zero error {zero_target - pos:+.3f}"
                )


def get_key(timeout=0.05):
    rlist, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        timeout,
    )

    if rlist:
        return sys.stdin.read(1)

    return None


def restore_terminal(old_settings):
    if old_settings is not None:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_settings,
        )


def main():
    node = None
    old_settings = None

    try:
        rclpy.init(
            signal_handler_options=SignalHandlerOptions.NO,
        )

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        node = KeyboardJointServo()

        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.01,
            )

            key = get_key()

            if key is None:
                continue

            if key == "x":
                node.publish_stop_commands()
                break

            if key == "p":
                node.print_positions()
                continue

            if key == "g":
                node.start_auto_move(
                    START_POSITION,
                    "start position",
                )
                continue

            if key == "h":
                node.start_auto_move(
                    ZERO_POSITION,
                    "zero position",
                )
                continue

            node.set_key_command(key)
            node.print_positions()

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping and restoring terminal...")

    finally:
        restore_terminal(old_settings)

        if node is not None:
            try:
                node.publish_stop_commands()
            except Exception:
                pass

            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
