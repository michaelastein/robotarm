#!/usr/bin/env python3

import math
import threading
import time

import numpy as np

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

PUBLISH_PERIOD = 0.01  # 100 Hz


# Input directly in base_link:
#   vx = base_link x
#   vy = base_link y
#   vz = base_link z
BASE_X_SIGN = 1.0
BASE_Y_SIGN = 1.0
BASE_Z_SIGN = 1.0


# Keep this small for diagnosis.
MAX_CART_VEL = 0.006
CART_DEADBAND = 0.0005

# Joint output limits.
MAX_JOINT_VEL = 0.10
JOINT_VEL_DEADBAND = 0.0001

# No smoothing, no minimum velocity.
DAMPING = 0.035
JACOBIAN_EPS = 1e-4

JOINT_LIMITS = {
    "base_joint": (-3.0, 3.0),
    "shoulder_joint": (-0.52359878, 1.39626340),
    "elbow_joint": (-0.69813170, 2.44346095),
}

JOINT_LIMIT_MARGIN = 0.05


def clamp(value, low, high):
    return max(low, min(high, value))


def rot_z(theta):
    c = math.cos(theta)
    s = math.sin(theta)

    return np.array(
        [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def rot_y(theta):
    c = math.cos(theta)
    s = math.sin(theta)

    return np.array(
        [
            [c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def trans(x, y, z):
    return np.array(
        [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def forward_kinematics(q):
    """
    FK from base_link to tool_tip_link using your URDF.
    """
    base = q[0]
    shoulder = q[1]
    elbow = q[2]

    T = np.eye(4, dtype=np.float64)

    # base_link -> column_link
    T = T @ trans(0.0, 0.0, 0.05)
    T = T @ rot_z(base)

    # column_link -> upper_arm_link
    T = T @ trans(0.0, 0.0, 0.12)
    T = T @ rot_y(shoulder)

    # upper_arm_link -> lower_arm_link
    T = T @ trans(-0.025, 0.0, 0.14)
    T = T @ rot_y(elbow)

    # lower_arm_link -> tool_tip_link
    T = T @ trans(0.0, 0.0, 0.15)

    return T


def tip_position(q):
    T = forward_kinematics(q)
    return T[0:3, 3].copy()


def numeric_position_jacobian(q):
    """
    3x3 position-only Jacobian:
      tip_velocity = J * qdot
    """
    J = np.zeros((3, 3), dtype=np.float64)

    p0 = tip_position(q)

    for i in range(3):
        qp = q.copy()
        qp[i] += JACOBIAN_EPS

        pp = tip_position(qp)

        J[:, i] = (pp - p0) / JACOBIAN_EPS

    return J


def damped_least_squares(J, v):
    """
    qdot = J^T (J J^T + lambda^2 I)^-1 v
    """
    lambda2 = DAMPING * DAMPING
    A = J @ J.T + lambda2 * np.eye(3, dtype=np.float64)

    try:
        qdot = J.T @ np.linalg.solve(A, v)
    except np.linalg.LinAlgError:
        qdot = np.zeros(3, dtype=np.float64)

    return qdot


class ToolPositionVelocityControl(Node):

    def __init__(self):
        super().__init__("tool_position_velocity_control")

        self.pub = self.create_publisher(
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
        self.current_positions = {}
        self.have_all_joints = False

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.last_debug_time = 0.0

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_command,
        )

        self.get_logger().warn(
            "DIAGNOSTIC MODE: v_xyz -> Jacobian -> qdot only"
        )
        self.get_logger().warn(
            "No target integrator, no correction, no smoothing, no min velocity in script"
        )
        self.get_logger().info(
            "Input is directly in base_link: vx vy vz"
        )

    def joint_state_callback(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.current_positions[name] = float(pos)

            self.have_all_joints = all(
                joint in self.current_positions
                for joint in JOINT_NAMES
            )

    def get_q(self):
        with self.lock:
            if not self.have_all_joints:
                return None

            return np.array(
                [
                    self.current_positions["base_joint"],
                    self.current_positions["shoulder_joint"],
                    self.current_positions["elbow_joint"],
                ],
                dtype=np.float64,
            )

    def publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.pub.publish(msg)

    def input_to_base_velocity(self):
        vx = clamp(self.vx, -MAX_CART_VEL, MAX_CART_VEL)
        vy = clamp(self.vy, -MAX_CART_VEL, MAX_CART_VEL)
        vz = clamp(self.vz, -MAX_CART_VEL, MAX_CART_VEL)

        if abs(vx) < CART_DEADBAND:
            vx = 0.0

        if abs(vy) < CART_DEADBAND:
            vy = 0.0

        if abs(vz) < CART_DEADBAND:
            vz = 0.0

        return np.array(
            [
                BASE_X_SIGN * vx,
                BASE_Y_SIGN * vy,
                BASE_Z_SIGN * vz,
            ],
            dtype=np.float64,
        )

    def apply_joint_limits(self, q, qdot):
        qdot_out = qdot.copy()

        for i, joint in enumerate(JOINT_NAMES):
            lower, upper = JOINT_LIMITS[joint]

            if q[i] <= lower + JOINT_LIMIT_MARGIN and qdot_out[i] < 0.0:
                qdot_out[i] = 0.0

            if q[i] >= upper - JOINT_LIMIT_MARGIN and qdot_out[i] > 0.0:
                qdot_out[i] = 0.0

        return qdot_out

    def postprocess_qdot(self, qdot):
        out = qdot.copy()

        for i in range(3):
            out[i] = clamp(out[i], -MAX_JOINT_VEL, MAX_JOINT_VEL)

            if abs(out[i]) < JOINT_VEL_DEADBAND:
                out[i] = 0.0

        return out

    def maybe_debug_print(self, q, p, J, v, qdot):
        now = time.monotonic()

        if now - self.last_debug_time < 0.5:
            return

        self.last_debug_time = now

        self.get_logger().info(
            "q=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "tip_fk=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "v=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "qdot=[{:+.5f}, {:+.5f}, {:+.5f}]".format(
                q[0], q[1], q[2],
                p[0], p[1], p[2],
                v[0], v[1], v[2],
                qdot[0], qdot[1], qdot[2],
            )
        )

        self.get_logger().info(
            "J rows: "
            "x=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "y=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "z=[{:+.4f}, {:+.4f}, {:+.4f}]".format(
                J[0, 0], J[0, 1], J[0, 2],
                J[1, 0], J[1, 1], J[1, 2],
                J[2, 0], J[2, 1], J[2, 2],
            )
        )

    def publish_command(self):
        q = self.get_q()

        if q is None:
            self.publish_zero()
            self.get_logger().warn(
                "Waiting for joint states...",
                throttle_duration_sec=1.0,
            )
            return

        v = self.input_to_base_velocity()

        if float(np.linalg.norm(v)) <= CART_DEADBAND:
            self.publish_zero()
            return

        p = tip_position(q)
        J = numeric_position_jacobian(q)

        qdot = damped_least_squares(J, v)
        qdot = self.apply_joint_limits(q, qdot)
        qdot = self.postprocess_qdot(qdot)

        msg = Float64MultiArray()
        msg.data = [
            float(qdot[0]),
            float(qdot[1]),
            float(qdot[2]),
        ]

        self.pub.publish(msg)

        self.maybe_debug_print(q, p, J, v, qdot)

    def input_loop(self):
        while rclpy.ok():
            text = input(
                "\nEnter vx vy vz in base_link "
                "(s stop, p print, q quit): "
            ).strip()

            if text == "q":
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.publish_zero()
                rclpy.shutdown()
                return

            if text == "s":
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.publish_zero()
                print("Stopped")
                continue

            if text == "p":
                q = self.get_q()

                if q is None:
                    print("Joint states unknown")
                    continue

                p = tip_position(q)
                J = numeric_position_jacobian(q)

                print("")
                print(
                    "q base={:+.4f} shoulder={:+.4f} elbow={:+.4f}".format(
                        q[0], q[1], q[2],
                    )
                )
                print(
                    "tip_fk x={:+.4f} y={:+.4f} z={:+.4f}".format(
                        p[0], p[1], p[2],
                    )
                )
                print("Jacobian:")
                print(J)
                continue

            try:
                vals = [float(v) for v in text.split()]

                if len(vals) != 3:
                    print("Need exactly 3 numbers: vx vy vz")
                    print("Example: 0.003 0 0")
                    continue

                self.vx = clamp(vals[0], -MAX_CART_VEL, MAX_CART_VEL)
                self.vy = clamp(vals[1], -MAX_CART_VEL, MAX_CART_VEL)
                self.vz = clamp(vals[2], -MAX_CART_VEL, MAX_CART_VEL)

                print(
                    f"Sending v=({self.vx:+.4f}, "
                    f"{self.vy:+.4f}, "
                    f"{self.vz:+.4f}) in base_link"
                )

            except Exception:
                print("Example: 0.003 0 0")


def main(args=None):
    rclpy.init(args=args)

    node = ToolPositionVelocityControl()

    threading.Thread(
        target=node.input_loop,
        daemon=True,
    ).start()

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
