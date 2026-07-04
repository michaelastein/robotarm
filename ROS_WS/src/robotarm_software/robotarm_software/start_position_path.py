#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

START_POSITION = [
    0.3,
    0.43,
    1.1,
]

HOME_POSITION = [
    0.0,
    0.0,
    0.0,
]

ACTION_NAME = (
    "/arm_trajectory_controller/"
    "follow_joint_trajectory"
)

TRAJECTORY_DURATION_SECONDS = 12


class StartPositionPath(Node):

    def __init__(self):
        super().__init__("start_position_path")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

    def move(self, target_position, target_name):
        self.get_logger().info(
            f"Waiting for {ACTION_NAME}"
        )

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Trajectory controller is not available"
            )
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [
            float(position)
            for position in target_position
        ]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(
            sec=TRAJECTORY_DURATION_SECONDS,
            nanosec=0,
        )

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().warn(
            f"Moving to {target_name}"
        )

        send_future = self.client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                "No goal handle received"
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "Trajectory goal was rejected"
            )
            return False

        self.get_logger().info(
            "Trajectory goal accepted"
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result_response = result_future.result()

        if result_response is None:
            self.get_logger().error(
                "No trajectory result received"
            )
            return False

        result = result_response.result

        if (
            result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error(
                "Trajectory failed with error code "
                f"{result.error_code}: "
                f"{result.error_string}"
            )
            return False

        self.get_logger().warn(
            f"Reached {target_name}"
        )

        return True


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Move the robot arm to a predefined "
            "joint position"
        )
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


def main(args=None):
    cli_args = parse_arguments()

    rclpy.init(args=args)

    node = StartPositionPath()
    success = False

    try:
        if cli_args.start:
            success = node.move(
                START_POSITION,
                "path-planning start position",
            )

        elif cli_args.zero:
            success = node.move(
                HOME_POSITION,
                "zero position",
            )

    except KeyboardInterrupt:
        node.get_logger().warn(
            "Movement interrupted"
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
