#!/usr/bin/env python3
"""
ROS 2 relay node for republishing joint states with a different QoS profile.

This node subscribes to the standard /joint_states topic and republishes each
sensor_msgs/msg/JointState message unchanged on /joint_states_moveit.

The relay is useful when the original publisher and the receiving application
use incompatible Quality of Service settings. In this configuration, the node
accepts reliable and transient-local joint-state messages, then republishes them
using best-effort and volatile delivery for consumers such as MoveIt-related
nodes.

ROS topics:
    INPUT_TOPIC:
        Source topic containing sensor_msgs/msg/JointState messages.

    OUTPUT_TOPIC:
        Destination topic on which the same JointState messages are republished.

Input QoS:
    ReliabilityPolicy.RELIABLE:
        Requests reliable delivery from the source publisher.

    DurabilityPolicy.TRANSIENT_LOCAL:
        Allows the subscription to receive the most recently retained sample
        from a compatible transient-local publisher.

    HistoryPolicy.KEEP_LAST:
        Stores only the newest configured number of messages.

    INPUT_QUEUE_DEPTH:
        Maximum number of recent input messages retained by the QoS history.

Output QoS:
    ReliabilityPolicy.BEST_EFFORT:
        Publishes without retransmission guarantees, which is commonly suitable
        for frequently updated sensor-state data.

    DurabilityPolicy.VOLATILE:
        Does not retain messages for subscribers that connect later.

    HistoryPolicy.KEEP_LAST:
        Stores only the newest configured number of outgoing messages.

    OUTPUT_QUEUE_DEPTH:
        Maximum number of recent output messages retained by the QoS history.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState


INPUT_TOPIC = "/joint_states"
OUTPUT_TOPIC = "/joint_states_moveit"

INPUT_QUEUE_DEPTH = 10
OUTPUT_QUEUE_DEPTH = 10


class JointStatesRelay(Node):
    """Relay JointState messages between topics with different QoS profiles."""

    def __init__(self) -> None:
        """
        Initialize the ROS 2 publisher, subscription, and QoS profiles.

        Parameters:
            None.

        Returns:
            None.
        """
        super().__init__("joint_states_relay")

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=INPUT_QUEUE_DEPTH,
        )

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=OUTPUT_QUEUE_DEPTH,
        )

        self.publisher = self.create_publisher(
            JointState,
            OUTPUT_TOPIC,
            output_qos,
        )

        self.subscription = self.create_subscription(
            JointState,
            INPUT_TOPIC,
            self.callback,
            input_qos,
        )

        self.get_logger().info(
            f"Relaying {INPUT_TOPIC} to {OUTPUT_TOPIC}"
        )

    def callback(self, message: JointState) -> None:
        """
        Republish one joint-state message without modifying its contents.

        Parameters:
            message:
                Incoming sensor_msgs/msg/JointState message. Its header, joint
                names, positions, velocities, and efforts are forwarded exactly
                as received.

        Returns:
            None.
        """
        self.publisher.publish(message)


def main(args=None) -> None:
    """
    Initialize ROS 2, run the relay node, and shut it down safely.

    Parameters:
        args:
            Optional command-line arguments passed to rclpy.init(). When None,
            rclpy uses the process command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = JointStatesRelay()

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