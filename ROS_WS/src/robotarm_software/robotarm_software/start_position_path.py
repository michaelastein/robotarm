#!/usr/bin/env python3
"""
ROS 2 command-line node for moving the robot arm to predefined joint positions.

The node sends one control_msgs/action/FollowJointTrajectory goal to the arm
trajectory controller, waits for completion, and exits with a process status
that indicates success or failure.

Command-line options:
    --start or -s:
        Move the arm to START_POSITION.

    --zero or -z:
        Move the arm to HOME_POSITION.

Exactly one option must be selected.

Joint configuration:
    JOINT_NAMES:
        Ordered list of joints expected by the trajectory controller.

    START_POSITION:
        Predefined path-planning start position in radians. Values correspond
        to JOINT_NAMES in the same order.

    HOME_POSITION:
        Zero or home joint position in radians. Values correspond to
        JOINT_NAMES in the same order.

Trajectory configuration:
    ACTION_NAME:
        FollowJointTrajectory action endpoint provided by the arm trajectory
        controller.

    TRAJECTORY_DURATION_SECONDS:
        Time allowed for the single trajectory point to be reached.

    ACTION_SERVER_TIMEOUT_SECONDS:
        Maximum time to wait for the trajectory action server to become
        available.

Exit status:
    0:
        The trajectory completed successfully.

    1:
        The action server was unavailable, the goal was rejected, execution
        failed, no result was received, or movement was interrupted.
"""

import argparse
from collections.abc import Sequence

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
ACTION_SERVER_TIMEOUT_SECONDS = 10.0


class StartPositionPath(Node):
    """Send predefined joint trajectories to the arm controller."""

    def __init__(self) -> None:
        """
        Initialize the ROS 2 node and trajectory action client.

        Parameters:
            None.

        Returns:
            None.
        """
        super().__init__("start_position_path")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

    def move(
        self,
        target_position: Sequence[float],
        target_name: str,
    ) -> bool:
        """
        Move the arm to one predefined joint position.

        Parameters:
            target_position:
                Sequence of joint angles in radians. The values must follow the
                same order as JOINT_NAMES.

            target_name:
                Human-readable name used in status and error messages.

        Returns:
            True when the trajectory controller accepts the goal and reports
            FollowJointTrajectory.Result.SUCCESSFUL.

            False when the action server is unavailable, the goal is rejected,
            no response is received, or trajectory execution fails.
        """
        if len(target_position) != len(JOINT_NAMES):
            self.get_logger().error(
                "Target position length does not match JOINT_NAMES"
            )
            return False

        self.get_logger().info(
            f"Waiting for {ACTION_NAME}"
        )

        if not self.client.wait_for_server(
            timeout_sec=ACTION_SERVER_TIMEOUT_SECONDS
        ):
            self.get_logger().error(
                "Trajectory controller is not available"
            )
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [
            float(position)
            for position in target_position
        ]
        point.velocities = [0.0] * len(JOINT_NAMES)
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
        rclpy.spin_until_future_complete(self, send_future)

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

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

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

        self.get_logger().info(
            f"Reached {target_name}"
        )
        return True


def parse_arguments() -> argparse.Namespace:
    """
    Parse the required start-or-zero command-line selection.

    Parameters:
        None. Arguments are read from the process command line.

    Returns:
        argparse.Namespace containing the boolean fields:
            start:
                True when --start or -s was selected.

            zero:
                True when --zero or -z was selected.
    """
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


def main(args=None) -> None:
    """
    Parse the command, initialize ROS 2, execute the move, and exit.

    Parameters:
        args:
            Optional ROS-specific command-line arguments passed to rclpy.init().
            The start/zero selection is parsed separately by argparse.

    Returns:
        None. The function terminates the process with exit status 0 on success
        and 1 on failure.
    """
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