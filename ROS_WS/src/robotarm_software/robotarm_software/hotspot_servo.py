#!/usr/bin/env python3

import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray


# ============================================================
# Frames / joints
# ============================================================

JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

COMMAND_TOPIC = "/servo_controller/commands"
HOTSPOT_TOPIC = "/hotspot/target"

PUBLISH_PERIOD = 0.01  # 100 Hz


# ============================================================
# Hotspot tracking parameters
# ============================================================
#
# /hotspot/target:
#   data[0] = visible, >= 0.5 means target visible
#   data[1] = err_x
#   data[2] = err_y
#   data[3] = confidence
#
# Desired mapping:
#
#   hotspot left:
#     err_x negative -> move arm left  -> +base_link y
#
#   hotspot right:
#     err_x positive -> move arm right -> -base_link y
#
#   hotspot lower:
#     err_y positive -> move arm back  -> -base_link x
#
#   hotspot upper:
#     err_y negative -> move arm front -> +base_link x

KP_IMAGE_X_TO_BASE_Y = 0.003
KP_IMAGE_Y_TO_BASE_X = 0.003

HOTSPOT_X_SIGN = -1.0
HOTSPOT_Y_SIGN = -1.0

CONF_MIN = 0.0

TARGET_TIMEOUT = 0.25
STOP_COMMANDS_AFTER_LOST = 10

# Slower filtering = less overshoot and less twitch.
SMOOTHING_ALPHA = 0.08
LOST_ALPHA = 0.20

COMMAND_EPSILON = 0.00002
COMMAND_MULTIPLIER = 1.0


# ============================================================
# Hysteresis center deadband
# ============================================================
#
# Once centered, stay stopped until error leaves EXIT band.
# This prevents oscillating around center.

CENTER_ENTER_DEADBAND_X = 0.18
CENTER_EXIT_DEADBAND_X = 0.28

CENTER_ENTER_DEADBAND_Y = 0.18
CENTER_EXIT_DEADBAND_Y = 0.28


# ============================================================
# Cartesian velocity limits
# ============================================================

MAX_CART_VEL = 0.0015

# Important:
# This deadband is only used for final "is command basically zero" checks.
# Do NOT use it to zero filtered_base_v every cycle, otherwise smoothing
# can never accumulate a small command.
CART_DEADBAND = 0.00002

ENABLE_BASE_X = True
ENABLE_BASE_Y = True
ENABLE_BASE_Z = False


# ============================================================
# IK / Jacobian parameters
# ============================================================

DAMPING = 0.035
JACOBIAN_EPS = 1e-4


# ============================================================
# Joint velocity output
# ============================================================

MAX_JOINT_VEL = 0.08

# Important:
# This must be low for hotspot following. Otherwise the small IK result
# gets erased before it reaches the hardware.
JOINT_VEL_DEADBAND = 0.0002

# Important for hotspot tracking:
# Do NOT force small qdot to ±0.08 here.
# Otherwise it overshoots the center all the time.
FORCE_MIN_JOINT_VEL_IN_THIS_SCRIPT = False
MIN_USEFUL_JOINT_VEL = 0.08

JOINT_SMOOTHING_ALPHA = 0.10


# ============================================================
# Joint limits
# ============================================================

JOINT_LIMITS = {
    "base_joint": (-3.0, 3.0),
    "shoulder_joint": (-0.52359878, 1.39626340),
    "elbow_joint": (-0.69813170, 2.44346095),
}

JOINT_LIMIT_MARGIN = 0.05


# ============================================================
# Debug
# ============================================================

DEBUG_PRINT_PERIOD = 1.0
DEBUG_PRINT_JOINT_STATE_ORDER = True
DEBUG_PRINT_JACOBIAN = False
DEBUG_PRINT_OBSERVED_JOINT_DELTA = True


def clamp(value, low, high):
    return max(low, min(high, value))


def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)

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
    c = np.cos(theta)
    s = np.sin(theta)

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


class HotspotDirectJointServo(Node):

    def __init__(self):
        super().__init__("hotspot_direct_joint_servo")

        self.target_sub = self.create_subscription(
            Float32MultiArray,
            HOTSPOT_TOPIC,
            self.target_callback,
            10,
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        self.lock = threading.Lock()

        self.target_visible = False
        self.err_x = 0.0
        self.err_y = 0.0
        self.conf = 0.0
        self.last_target_time = None

        self.current_positions = {}
        self.have_all_joints = False

        self.filtered_base_v = np.zeros(3, dtype=np.float64)
        self.filtered_qdot = np.zeros(3, dtype=np.float64)
        self.cmd_qdot = np.zeros(3, dtype=np.float64)

        self.centered_x = False
        self.centered_y = False

        self.lost_stop_publish_count = STOP_COMMANDS_AFTER_LOST

        self.printed_joint_state_order = False

        self.last_debug_time = 0.0
        self.last_tip_pos = None

        self.last_q_for_observed_delta = None
        self.last_q_time_for_observed_delta = None

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_command,
        )

        self.get_logger().info("Hotspot direct joint servo started")
        self.get_logger().info(f"Listening to {HOTSPOT_TOPIC}")
        self.get_logger().info(f"Publishing direct joint velocities to {COMMAND_TOPIC}")
        self.get_logger().warn("MoveIt Servo is bypassed/removed.")
        self.get_logger().warn("Soft hotspot mode: no forced min joint velocity.")
        self.get_logger().info("Command order is fixed: [base_joint, shoulder_joint, elbow_joint]")
        self.get_logger().info("Mapping: err_x negative -> +base_y left")
        self.get_logger().info("Mapping: err_y positive -> -base_x back")
        self.get_logger().warn("base_link z tracking is disabled.")

    # ========================================================
    # Callbacks
    # ========================================================

    def target_callback(self, msg):
        if len(msg.data) < 4:
            return

        with self.lock:
            self.target_visible = msg.data[0] >= 0.5
            self.err_x = float(msg.data[1])
            self.err_y = float(msg.data[2])
            self.conf = float(msg.data[3])
            self.last_target_time = time.monotonic()

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

    # ========================================================
    # State helpers
    # ========================================================

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

    def target_is_fresh(self):
        if self.last_target_time is None:
            return False

        return (time.monotonic() - self.last_target_time) <= TARGET_TIMEOUT

    # ========================================================
    # Command helpers
    # ========================================================

    def publish_zero_once(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.cmd_pub.publish(msg)

    def apply_hysteresis_deadband(self, err_x, err_y):
        """
        Center hysteresis:
        - If already centered, remain centered until EXIT threshold.
        - If not centered, enter centered when below ENTER threshold.
        """

        if self.centered_x:
            if abs(err_x) > CENTER_EXIT_DEADBAND_X:
                self.centered_x = False
        else:
            if abs(err_x) < CENTER_ENTER_DEADBAND_X:
                self.centered_x = True

        if self.centered_y:
            if abs(err_y) > CENTER_EXIT_DEADBAND_Y:
                self.centered_y = False
        else:
            if abs(err_y) < CENTER_ENTER_DEADBAND_Y:
                self.centered_y = True

        if self.centered_x:
            err_x = 0.0

        if self.centered_y:
            err_y = 0.0

        return err_x, err_y

    def hotspot_to_base_velocity(self, active, err_x, err_y, conf):
        raw_v = np.zeros(3, dtype=np.float64)

        if not active:
            self.centered_x = False
            self.centered_y = False
            return raw_v

        if conf < CONF_MIN:
            return raw_v

        err_x, err_y = self.apply_hysteresis_deadband(err_x, err_y)

        # Desired behavior:
        #
        #   hotspot left:
        #     err_x negative -> +base_y left
        #
        #   hotspot right:
        #     err_x positive -> -base_y right
        #
        #   hotspot lower:
        #     err_y positive -> -base_x back
        #
        #   hotspot upper:
        #     err_y negative -> +base_x front

        vx = HOTSPOT_Y_SIGN * KP_IMAGE_Y_TO_BASE_X * err_y
        vy = HOTSPOT_X_SIGN * KP_IMAGE_X_TO_BASE_Y * err_x
        vz = 0.0

        if not ENABLE_BASE_X:
            vx = 0.0

        if not ENABLE_BASE_Y:
            vy = 0.0

        if not ENABLE_BASE_Z:
            vz = 0.0

        vx = clamp(vx, -MAX_CART_VEL, MAX_CART_VEL)
        vy = clamp(vy, -MAX_CART_VEL, MAX_CART_VEL)
        vz = clamp(vz, -MAX_CART_VEL, MAX_CART_VEL)

        # Do not apply CART_DEADBAND here.
        # Hysteresis already decides if image error is centered.
        # Small nonzero velocities must survive smoothing.

        raw_v[:] = [vx, vy, vz]

        return raw_v

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
        For hotspot following:
        - clamp tiny qdot to zero
        - optionally scale whole vector to useful speed
        - but by default DO NOT force min joint speed
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

    # ========================================================
    # Debug
    # ========================================================

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
        active,
        q,
        tip,
        raw_base_v,
        filtered_base_v,
        qdot_raw,
        qdot_limited,
        qdot_cmd,
        J,
        did_publish,
        reason,
    ):
        now = time.monotonic()

        if now - self.last_debug_time < DEBUG_PRINT_PERIOD:
            return

        self.last_debug_time = now

        if self.last_tip_pos is None:
            dx = 0.0
            dy = 0.0
            dz = 0.0
        else:
            dx = tip[0] - self.last_tip_pos[0]
            dy = tip[1] - self.last_tip_pos[1]
            dz = tip[2] - self.last_tip_pos[2]

        self.last_tip_pos = tip.copy()

        predicted_tip_v = J @ qdot_cmd
        observed_text = self.observed_joint_delta_text(q)

        self.get_logger().info(
            "tip=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "d_tip=[{:+.4f}, {:+.4f}, {:+.4f}] "
            "active={} visible={} centered=({}, {}) "
            "err=({:+.3f},{:+.3f}) conf={:.2f} "
            "raw_base_v=[{:+.5f}, {:+.5f}, {:+.5f}] "
            "filtered_base_v=[{:+.5f}, {:+.5f}, {:+.5f}] "
            "q=[{:+.3f}, {:+.3f}, {:+.3f}] "
            "qdot_raw=[{:+.5f}, {:+.5f}, {:+.5f}] "
            "qdot_limited=[{:+.5f}, {:+.5f}, {:+.5f}] "
            "qdot_cmd=[{:+.5f}, {:+.5f}, {:+.5f}] "
            "pred_tip_v=[{:+.5f}, {:+.5f}, {:+.5f}] "
            "{} published={} reason={}".format(
                tip[0], tip[1], tip[2],
                dx, dy, dz,
                active,
                self.target_visible,
                self.centered_x,
                self.centered_y,
                self.err_x,
                self.err_y,
                self.conf,
                raw_base_v[0], raw_base_v[1], raw_base_v[2],
                filtered_base_v[0], filtered_base_v[1], filtered_base_v[2],
                q[0], q[1], q[2],
                qdot_raw[0], qdot_raw[1], qdot_raw[2],
                qdot_limited[0], qdot_limited[1], qdot_limited[2],
                qdot_cmd[0], qdot_cmd[1], qdot_cmd[2],
                predicted_tip_v[0], predicted_tip_v[1], predicted_tip_v[2],
                observed_text,
                did_publish,
                reason,
            )
        )

        if DEBUG_PRINT_JACOBIAN:
            self.get_logger().info(
                "\nJacobian columns: tip_velocity = J * qdot\n"
                "  base     dx dy dz = [{:+.5f}, {:+.5f}, {:+.5f}]\n"
                "  shoulder dx dy dz = [{:+.5f}, {:+.5f}, {:+.5f}]\n"
                "  elbow    dx dy dz = [{:+.5f}, {:+.5f}, {:+.5f}]".format(
                    J[0, 0], J[1, 0], J[2, 0],
                    J[0, 1], J[1, 1], J[2, 1],
                    J[0, 2], J[1, 2], J[2, 2],
                )
            )

    # ========================================================
    # Main control loop
    # ========================================================

    def publish_command(self):
        q = self.get_q()

        if q is None:
            self.publish_zero_once()
            self.get_logger().warn(
                "Waiting for joint states...",
                throttle_duration_sec=1.0,
            )
            return

        with self.lock:
            target_visible = self.target_visible
            err_x = self.err_x
            err_y = self.err_y
            conf = self.conf
            fresh = self.target_is_fresh()

        active = target_visible and fresh and conf >= CONF_MIN

        raw_base_v = self.hotspot_to_base_velocity(
            active,
            err_x,
            err_y,
            conf,
        )

        alpha = SMOOTHING_ALPHA if active else LOST_ALPHA

        self.filtered_base_v = (
            alpha * raw_base_v
            + (1.0 - alpha) * self.filtered_base_v
        )

        # Clamp filtered velocity, but DO NOT deadband it here.
        # Deadband here caused the command to stay zero forever.
        for i in range(3):
            self.filtered_base_v[i] = clamp(
                self.filtered_base_v[i] * COMMAND_MULTIPLIER,
                -MAX_CART_VEL,
                MAX_CART_VEL,
            )

        tip = tip_position(q)
        J = numeric_position_jacobian(q)

        if not active:
            self.filtered_base_v[:] = 0.0
            self.filtered_qdot[:] = 0.0
            self.cmd_qdot[:] = 0.0

            if self.lost_stop_publish_count < STOP_COMMANDS_AFTER_LOST:
                self.publish_zero_once()
                self.lost_stop_publish_count += 1

                self.maybe_debug_print(
                    active,
                    q,
                    tip,
                    raw_base_v,
                    self.filtered_base_v,
                    np.zeros(3, dtype=np.float64),
                    np.zeros(3, dtype=np.float64),
                    np.zeros(3, dtype=np.float64),
                    J,
                    True,
                    "lost_or_stale_zero_stop",
                )
            else:
                self.maybe_debug_print(
                    active,
                    q,
                    tip,
                    raw_base_v,
                    self.filtered_base_v,
                    np.zeros(3, dtype=np.float64),
                    np.zeros(3, dtype=np.float64),
                    np.zeros(3, dtype=np.float64),
                    J,
                    False,
                    "lost_or_stale_no_publish",
                )

            return

        if float(np.linalg.norm(self.filtered_base_v)) <= CART_DEADBAND:
            self.filtered_qdot[:] = 0.0
            self.cmd_qdot[:] = 0.0

            self.maybe_debug_print(
                active,
                q,
                tip,
                raw_base_v,
                self.filtered_base_v,
                np.zeros(3, dtype=np.float64),
                np.zeros(3, dtype=np.float64),
                np.zeros(3, dtype=np.float64),
                J,
                False,
                "active_but_tiny_no_publish",
            )
            return

        qdot_raw = damped_least_squares(
            J,
            self.filtered_base_v,
        )

        qdot_limited = self.apply_joint_limits(q, qdot_raw)
        qdot_limited = self.postprocess_qdot(qdot_limited)

        self.filtered_qdot = (
            JOINT_SMOOTHING_ALPHA * qdot_limited
            + (1.0 - JOINT_SMOOTHING_ALPHA) * self.filtered_qdot
        )

        self.filtered_qdot = self.postprocess_qdot(self.filtered_qdot)

        self.cmd_qdot = self.filtered_qdot.copy()

        if (
            abs(self.cmd_qdot[0]) < COMMAND_EPSILON
            and abs(self.cmd_qdot[1]) < COMMAND_EPSILON
            and abs(self.cmd_qdot[2]) < COMMAND_EPSILON
        ):
            self.maybe_debug_print(
                active,
                q,
                tip,
                raw_base_v,
                self.filtered_base_v,
                qdot_raw,
                qdot_limited,
                self.cmd_qdot,
                J,
                False,
                "active_but_tiny_qdot_no_publish",
            )
            return

        msg = Float64MultiArray()
        msg.data = [
            float(self.cmd_qdot[0]),
            float(self.cmd_qdot[1]),
            float(self.cmd_qdot[2]),
        ]

        self.cmd_pub.publish(msg)
        self.lost_stop_publish_count = 0

        self.maybe_debug_print(
            active,
            q,
            tip,
            raw_base_v,
            self.filtered_base_v,
            qdot_raw,
            qdot_limited,
            self.cmd_qdot,
            J,
            True,
            "active_tracking_direct_joint",
        )


def main(args=None):
    rclpy.init(args=args)

    node = HotspotDirectJointServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero_once()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
