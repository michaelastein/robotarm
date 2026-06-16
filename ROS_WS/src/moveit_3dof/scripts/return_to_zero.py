#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ServoCommandType


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

TARGET = {
    "base_joint": 0.0,
    "shoulder_joint": 0.0,
    "elbow_joint": 0.0,
}

KP = 0.8
MAX_VEL = 0.12
MIN_VEL = 0.025
TOLERANCE = 0.025
PUBLISH_RATE = 50.0


def clamp(value, low, high):
    return max(low, min(high, value))


class ReturnToZero(Node):
    def __init__(self):
        super().__init__("return_to_zero")

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

        self.positions = {}
        self.done = False

        self.switch_to_joint_jog()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.update,
        )

        self.get_logger().info("Returning robot to 0 0 0...")

    def switch_to_joint_jog(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /servo_node/switch_command_type...")

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("MoveIt Servo switched to JOINT_JOG mode")
        else:
            self.get_logger().error("Failed to switch MoveIt Servo to JOINT_JOG mode")

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.positions[name] = pos

    def publish_zero(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = JOINT_NAMES
        msg.velocities = [0.0, 0.0, 0.0]
        msg.duration = 0.1
        self.pub.publish(msg)

    def update(self):
        if self.done:
            self.publish_zero()
            return

        if not all(joint in self.positions for joint in JOINT_NAMES):
            self.get_logger().warn(
                "Waiting for all joint states...",
                throttle_duration_sec=1.0,
            )
            return

        velocities = []
        all_reached = True

        for joint in JOINT_NAMES:
            error = TARGET[joint] - self.positions[joint]

            if abs(error) > TOLERANCE:
                all_reached = False
                vel = KP * error
                vel = clamp(vel, -MAX_VEL, MAX_VEL)

                if abs(vel) < MIN_VEL:
                    vel = MIN_VEL if vel > 0.0 else -MIN_VEL
            else:
                vel = 0.0

            velocities.append(vel)

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = JOINT_NAMES
        msg.velocities = velocities
        msg.duration = 0.1
        self.pub.publish(msg)

        status = " ".join(
            [
                f"{joint}: {self.positions[joint]:+.3f}"
                for joint in JOINT_NAMES
            ]
        )

        self.get_logger().info(
            f"vel={['%.3f' % v for v in velocities]} | {status}",
            throttle_duration_sec=0.5,
        )

        if all_reached:
            self.done = True
            for _ in range(10):
                self.publish_zero()

            self.get_logger().info("Reached 0 0 0. Stopping.")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = ReturnToZero()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_zero()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
