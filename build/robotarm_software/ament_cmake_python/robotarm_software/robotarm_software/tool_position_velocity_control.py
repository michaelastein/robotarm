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

COMMAND_TOPIC = "/velocity_controller/commands"

PUBLISH_PERIOD = 0.01  # 100 Hz


# ============================================================
# Coordinate convention
# ============================================================

BASE_X_SIGN = 1.0
BASE_Y_SIGN = 1.0
BASE_Z_SIGN = 1.0


# ============================================================
# Cartesian motion parameters
# ============================================================

MAX_CART_VEL = 0.004
CART_DEADBAND = 0.0005

# Reiner IK-Test: keine Positionskorrektur
KP_POSITION = 0.0
MAX_CORRECTION_VEL = 0.0

MAX_TRACKING_ERROR = 0.035

DAMPING = 0.035
JACOBIAN_EPS = 1e-4


# ============================================================
# Joint velocity output
# ============================================================

MAX_JOINT_VEL = 0.08
JOINT_VEL_DEADBAND = 0.004

# Wichtig:
# Vektor-Skalierung, NICHT jedes Gelenk einzeln auf ±0.08 zwingen.
FORCE_MIN_JOINT_VEL_IN_THIS_SCRIPT = True
MIN_USEFUL_JOINT_VEL = 0.08

SMOOTHING_ALPHA = 0.30


# ============================================================
# Debug
# ============================================================

DEBUG_PRINT_PERIOD = 1.0
DEBUG_PRINT_JACOBIAN = True
DEBUG_PRINT_JOINT_STATE_ORDER = True
DEBUG_PRINT_OBSERVED_JOINT_DELTA = True


# ============================================================
# Joint limits
# ============================================================

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
    FK from base_link to tool_tip_link.

    q order:
      q[0] = base_joint
      q[1] = shoulder_joint
      q[2] = elbow_joint
    """
    base = q[0]
    shoulder = q[1]
    elbow = q[2]

    T = np.eye(4, dtype=np.float64)

    T = T @ trans(0.0, 0.0, 0.05)
    T = T @ rot_z(base)

    T = T @ trans(0.0, 0.0, 0.12)
    T = T @ rot_y(shoulder)

    T = T @ trans(-0.025, 0.0, 0.14)
    T = T @ rot_y(elbow)

    T = T @ trans(0.0, 0.0, 0.15)

    return T


def tip_position(q):
    T = forward_kinematics(q)
    return T[0:3, 3].copy()


def numeric_position_jacobian(q):
    J = np.zeros((3, 3), dtype=np.float64)
    p0 = tip_position(q)

    for i in range(3):
        qp = q.copy()
        qp[i] += JACOBIAN_EPS
        pp = tip_position(qp)
        J[:, i] = (pp - p0) / JACOBIAN_EPS

    return J


def damped_least_squares(J, v):
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

        self.target_position = None

        self.filtered_qdot = np.zeros(3, dtype=np.float64)

        self.last_time = time.monotonic()
        self.last_debug_time = 0.0

        self.last_q_for_observed_delta = None
        self.last_q_time_for_observed_delta = None

        self.printed_joint_state_order = False

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_command,
        )

        self.get_logger().info("Straight-axis base_link tool controller started")
        self.get_logger().info("Publishes directly to /velocity_controller/commands")
        self.get_logger().warn("MoveIt Servo is bypassed.")
        self.get_logger().warn("PURE IK TEST MODE: KP_POSITION=0.0, MAX_CORRECTION_VEL=0.0")
        self.get_logger().warn("VECTOR SCALE MODE: qdot direction preserved, vector scaled to useful motor speed")
        self.get_logger().info("Input: vx vy vz directly in base_link")
        self.get_logger().info("vx=base x front/back, vy=base y left/right, vz=base z up/down")
        self.get_logger().info("Command order is fixed: [base_joint, shoulder_joint, elbow_joint]")

    def joint_state_callback(self, msg):
        with self.lock:
            if DEBUG_PRINT_JOINT_STATE_ORDER and not self.printed_joint_state_order:
                self.printed_joint_state_order = True
                self.get_logger().warn(
                    "Received /joint_states order: "
                    + ", ".join(msg.name)
                )
                self.get_logger().warn(
                    "This node reads joint states BY NAME, not by index."
                )

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
                self.get_logger().warn(
                    f"{joint} near lower limit: q={q[i]:+.3f}, blocking negative qdot",
                    throttle_duration_sec=1.0,
                )
                qdot_out[i] = 0.0

            if q[i] >= upper - JOINT_LIMIT_MARGIN and qdot_out[i] > 0.0:
                self.get_logger().warn(
                    f"{joint} near upper limit: q={q[i]:+.3f}, blocking positive qdot",
                    throttle_duration_sec=1.0,
                )
                qdot_out[i] = 0.0

        return qdot_out

    def postprocess_qdot(self, qdot):
        """
        Vektor-Skalierung:

        Beispiel:
          qdot_raw = [0.000, +0.024, -0.029]

        Nicht:
          [0.000, +0.080, -0.080]

        Sondern:
          [0.000, +0.066, -0.080]

        Dadurch bleibt die kartesische Richtung besser erhalten.
        """
        out = qdot.copy()

        for i in range(3):
            if abs(out[i]) < JOINT_VEL_DEADBAND:
                out[i] = 0.0

        max_abs = float(np.max(np.abs(out)))

        if max_abs <= 0.0:
            return out

        if FORCE_MIN_JOINT_VEL_IN_THIS_SCRIPT:
            if max_abs < MIN_USEFUL_JOINT_VEL:
                scale = MIN_USEFUL_JOINT_VEL / max_abs
                out *= scale

        max_abs = float(np.max(np.abs(out)))

        if max_abs > MAX_JOINT_VEL:
            scale = MAX_JOINT_VEL / max_abs
            out *= scale

        return out

    def reset_target_to_current(self, current_position):
        self.target_position = current_position.copy()
        self.get_logger().info(
            "Target reset to current tip position: "
            f"x={self.target_position[0]:+.4f}, "
            f"y={self.target_position[1]:+.4f}, "
            f"z={self.target_position[2]:+.4f}"
        )

    def update_target_position(self, current_position, desired_base_velocity):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        dt = clamp(dt, 0.0, 0.05)

        if self.target_position is None:
            self.reset_target_to_current(current_position)

        tracking_error = self.target_position - current_position
        tracking_error_norm = float(np.linalg.norm(tracking_error))

        if tracking_error_norm > MAX_TRACKING_ERROR:
            self.get_logger().warn(
                f"Tracking error too large ({tracking_error_norm:.3f} m). "
                "Resetting target to current position for debug.",
                throttle_duration_sec=0.5,
            )
            self.target_position = current_position.copy()
            return

        self.target_position += desired_base_velocity * dt

    def compute_cartesian_control_velocity(
        self,
        current_position,
        desired_base_velocity,
    ):
        if self.target_position is None:
            self.reset_target_to_current(current_position)

        error = self.target_position - current_position

        correction = KP_POSITION * error

        correction_norm = float(np.linalg.norm(correction))

        if MAX_CORRECTION_VEL <= 0.0:
            correction[:] = 0.0
        elif correction_norm > MAX_CORRECTION_VEL:
            correction *= MAX_CORRECTION_VEL / (correction_norm + 1e-9)

        v_control = desired_base_velocity + correction

        v_norm = float(np.linalg.norm(v_control))
        max_total = MAX_CART_VEL + MAX_CORRECTION_VEL

        if max_total > 0.0 and v_norm > max_total:
            v_control *= max_total / (v_norm + 1e-9)

        return v_control, error, correction

    def observed_joint_delta_text(self, q):
        if not DEBUG_PRINT_OBSERVED_JOINT_DELTA:
            return ""

        now = time.monotonic()

        if self.last_q_for_observed_delta is None:
            self.last_q_for_observed_delta = q.copy()
            self.last_q_time_for_observed_delta = now
            return "observed_qdot=[init]"

        dt = now - self.last_q_time_for_observed_delta

        if dt <= 1e-6:
            return "observed_qdot=[dt too small]"

        dq = q - self.last_q_for_observed_delta
        observed_qdot = dq / dt

        self.last_q_for_observed_delta = q.copy()
        self.last_q_time_for_observed_delta = now

        return (
            "observed_qdot=[{:+.3f}, {:+.3f}, {:+.3f}]".format(
                observed_qdot[0],
                observed_qdot[1],
                observed_qdot[2],
            )
        )

    def maybe_debug_print(
        self,
        q,
        p,
        target,
        desired_v,
        control_v,
        correction,
        error,
        J,
        qdot_raw,
        qdot_limited,
        qdot_filtered,
    ):
        now = time.monotonic()

        if now - self.last_debug_time < DEBUG_PRINT_PERIOD:
            return

        self.last_debug_time = now

        if target is None:
            target_text = "None"
        else:
            target_text = (
                f"[{target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}]"
            )

        observed_text = self.observed_joint_delta_text(q)
        predicted_tip_v = J @ qdot_filtered

        self.get_logger().info(
            "q=[{:+.3f}, {:+.3f}, {:+.3f}] "
            "tip=[{:+.3f}, {:+.3f}, {:+.3f}] "
            "target={} "
            "desired_v=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "correction=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "control_v=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "err=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "qdot_raw=[{:+.3f}, {:+.3f}, {:+.3f}] "
            "qdot_limited=[{:+.3f}, {:+.3f}, {:+.3f}] "
            "qdot_cmd=[{:+.3f}, {:+.3f}, {:+.3f}] "
            "{}".format(
                q[0], q[1], q[2],
                p[0], p[1], p[2],
                target_text,
                desired_v[0], desired_v[1], desired_v[2],
                correction[0], correction[1], correction[2],
                control_v[0], control_v[1], control_v[2],
                error[0], error[1], error[2],
                qdot_raw[0], qdot_raw[1], qdot_raw[2],
                qdot_limited[0], qdot_limited[1], qdot_limited[2],
                qdot_filtered[0], qdot_filtered[1], qdot_filtered[2],
                observed_text,
            )
        )

        if DEBUG_PRINT_JACOBIAN:
            self.get_logger().info(
                "\nJacobian columns: tip_velocity = J * qdot\n"
                "  base     dx dy dz = [{:+.4f}, {:+.4f}, {:+.4f}]\n"
                "  shoulder dx dy dz = [{:+.4f}, {:+.4f}, {:+.4f}]\n"
                "  elbow    dx dy dz = [{:+.4f}, {:+.4f}, {:+.4f}]".format(
                    J[0, 0], J[1, 0], J[2, 0],
                    J[0, 1], J[1, 1], J[2, 1],
                    J[0, 2], J[1, 2], J[2, 2],
                )
            )

            self.get_logger().info(
                "predicted_tip_v_from_cmd=[{:+.4f}, {:+.4f}, {:+.4f}]".format(
                    predicted_tip_v[0],
                    predicted_tip_v[1],
                    predicted_tip_v[2],
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

        current_position = tip_position(q)
        desired_base_velocity = self.input_to_base_velocity()

        if float(np.linalg.norm(desired_base_velocity)) <= CART_DEADBAND:
            self.filtered_qdot[:] = 0.0
            self.target_position = None
            self.publish_zero()
            self.last_time = time.monotonic()
            return

        self.update_target_position(
            current_position,
            desired_base_velocity,
        )

        v_control, error, correction = self.compute_cartesian_control_velocity(
            current_position,
            desired_base_velocity,
        )

        J = numeric_position_jacobian(q)

        qdot_raw = damped_least_squares(J, v_control)

        qdot_limited = self.apply_joint_limits(q, qdot_raw)
        qdot_limited = self.postprocess_qdot(qdot_limited)

        self.filtered_qdot = (
            SMOOTHING_ALPHA * qdot_limited
            + (1.0 - SMOOTHING_ALPHA) * self.filtered_qdot
        )

        self.filtered_qdot = self.postprocess_qdot(self.filtered_qdot)

        msg = Float64MultiArray()
        msg.data = [
            float(self.filtered_qdot[0]),
            float(self.filtered_qdot[1]),
            float(self.filtered_qdot[2]),
        ]

        self.pub.publish(msg)

        self.maybe_debug_print(
            q,
            current_position,
            self.target_position,
            desired_base_velocity,
            v_control,
            correction,
            error,
            J,
            qdot_raw,
            qdot_limited,
            self.filtered_qdot,
        )

    def input_loop(self):
        while rclpy.ok():
            text = input(
                "\nEnter vx vy vz "
                "(base_link: x=front/back, y=left/right, z=up | "
                "s stop, p print, r reset target, q quit): "
            ).strip()

            if text == "q":
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.filtered_qdot[:] = 0.0
                self.target_position = None
                self.publish_zero()
                rclpy.shutdown()
                return

            if text == "s":
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.filtered_qdot[:] = 0.0
                self.target_position = None
                self.publish_zero()
                print("Stopped")
                continue

            if text == "r":
                q = self.get_q()

                if q is None:
                    print("Joint states unknown")
                    continue

                p = tip_position(q)
                self.reset_target_to_current(p)
                print("Target reset")
                continue

            if text == "p":
                q = self.get_q()

                if q is None:
                    print("Joint states unknown")
                    continue

                p = tip_position(q)

                if self.target_position is None:
                    target_text = "None"
                else:
                    target_text = (
                        f"x={self.target_position[0]:+.4f} "
                        f"y={self.target_position[1]:+.4f} "
                        f"z={self.target_position[2]:+.4f}"
                    )

                print(
                    "q base={:+.4f} shoulder={:+.4f} elbow={:+.4f} | "
                    "tip x={:+.4f} y={:+.4f} z={:+.4f} | "
                    "target {}".format(
                        q[0], q[1], q[2],
                        p[0], p[1], p[2],
                        target_text,
                    )
                )
                continue

            try:
                vals = [float(v) for v in text.split()]

                if len(vals) != 3:
                    print("Need exactly 3 numbers: vx vy vz")
                    print("Example: 0.004 0 0")
                    continue

                self.vx = clamp(vals[0], -MAX_CART_VEL, MAX_CART_VEL)
                self.vy = clamp(vals[1], -MAX_CART_VEL, MAX_CART_VEL)
                self.vz = clamp(vals[2], -MAX_CART_VEL, MAX_CART_VEL)

                print(
                    f"Sending base_link Cartesian v="
                    f"({self.vx:+.4f}, "
                    f"{self.vy:+.4f}, "
                    f"{self.vz:+.4f}) m/s "
                    f"[x=front/back, y=left/right, z=up]"
                )

            except Exception:
                print("Example: 0.004 0 0")


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
    finally:
        if rclpy.ok():
            node.publish_zero()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
