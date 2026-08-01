#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import JointState


class JointStatesRelay(Node):
    def __init__(self):
        super().__init__("joint_states_relay")

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states_moveit",
            output_qos,
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            input_qos,
        )

        self.get_logger().info(
            "Relaying /joint_states to /joint_states_moveit"
        )

    def callback(self, message):
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = JointStatesRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
