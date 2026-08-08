#!/usr/bin/env python3
"""
Direct joint-velocity controller for centering a detected image hotspot.

The node receives normalized image-space hotspot errors, converts them into a
Cartesian tool-tip velocity, and maps that velocity to joint velocities using
a numerically calculated Jacobian and damped least-squares inverse kinematics.

The Z coordinate is treated as a fixed reference captured when valid tracking
starts. The controller allows approximately +/-1 cm deviation around that
initial height. Outside this band, Z correction increases smoothly. If the Z
error becomes large, X/Y motion is reduced or stopped so the original height
can be recovered.

Important:
    The stored Z reference is NOT updated continuously. Height error is always
    measured relative to the original tracking-start height.
"""

import math
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray


# ---------------------------------------------------------------------------
# Joints / topics
# ---------------------------------------------------------------------------

JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
]

HOTSPOT_TOPIC = "/hotspot/target"
COMMAND_TOPIC = "/velocity_controller/commands"

PUBLISH_PERIOD = 0.01  # 100 Hz


# ---------------------------------------------------------------------------
# Hotspot input format
# ---------------------------------------------------------------------------

# /hotspot/target:
#
# data[0] = visible, >= 0.5 means visible
# data[1] = err_x
# data[2] = err_y
# data[3] = confidence
#
# Mapping:
#
# hotspot left:
# err_x negative -> move arm left -> +base_link y
#
# hotspot right:
# err_x positive -> move arm right -> -base_link y
#
# hotspot lower:
# err_y positive -> move arm back -> -base_link x
#
# hotspot upper:
# err_y negative -> move arm front -> +base_link x


# ---------------------------------------------------------------------------
# Hotspot centering control
# ---------------------------------------------------------------------------

CONF_MIN = 0.0
TARGET_TIMEOUT = 0.25

CENTER_ENTER_DEADBAND_X = 0.15
CENTER_EXIT_DEADBAND_X = 0.15

CENTER_ENTER_DEADBAND_Y = 0.15
CENTER_EXIT_DEADBAND_Y = 0.15

MAX_CART_VEL_XY = 0.004
MIN_EFFECTIVE_CART_VEL_XY = 0.0007

ERROR_FULL_SPEED = 1.00

HOTSPOT_X_SIGN_TO_BASE_Y = -1.0
HOTSPOT_Y_SIGN_TO_BASE_X = -1.0

ENABLE_BASE_X = True
ENABLE_BASE_Y = True


# ---------------------------------------------------------------------------
# Z height hold
# ---------------------------------------------------------------------------

ENABLE_Z_HOLD = True

# +/- 1 cm around the ORIGINAL tracking-start height is accepted.
Z_HOLD_DEADBAND = 0.010

# At 2 cm error, Z correction reaches its strong / maximum region.
Z_HOLD_HARD_BAND = 0.020

# Z feedback gain.
KZ_HOLD = 0.12

# Maximum correction speed in Z.
MAX_Z_HOLD_VEL = 0.0020

# XY remains unrestricted up to this Z error.
Z_SOFT_ERROR = 0.015

# At this Z error, XY is completely stopped.
Z_HARD_ERROR = 0.020


# ---------------------------------------------------------------------------
# Filtering / stopping
# ---------------------------------------------------------------------------

SMOOTHING_ALPHA = 0.18
LOST_ALPHA = 0.35

STOP_COMMANDS_AFTER_LOST = 10

COMMAND_EPSILON = 0.00001


# ---------------------------------------------------------------------------
# IK / Jacobian
# ---------------------------------------------------------------------------

DAMPING = 0.035
JACOBIAN_EPS = 1e-4


# ---------------------------------------------------------------------------
# Joint velocity output
# ---------------------------------------------------------------------------

MAX_JOINT_VEL = 0.3

# Whole-vector minimum preserves the IK joint ratio.
USE_MIN_EFFECTIVE_QDOT_VECTOR = True
MIN_EFFECTIVE_QDOT_VECTOR = 0.03

# IMPORTANT:
# Do not independently lift individual joints because that changes the
# direction of the IK solution and can create unwanted Z motion.
USE_MIN_EFFECTIVE_QDOT_PER_JOINT = False
MIN_EFFECTIVE_QDOT_PER_JOINT = 0.0035

JOINT_VEL_DEADBAND = 0.00005
JOINT_SMOOTHING_ALPHA = 0.20


# ---------------------------------------------------------------------------
# Joint limits
# ---------------------------------------------------------------------------

JOINT_LIMITS = {
    "base_joint": (-3.0, 3.0),
    "shoulder_joint": (-0.52359878, 1.39626340),
    "elbow_joint": (-0.69813170, 2.44346095),
}

JOINT_LIMIT_MARGIN = 0.05


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

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
    Compute tool-tip pose relative to base_link.
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
    """
    Numerical translational Jacobian.

    Maps:
        qdot -> [vx, vy, vz]
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
    Damped least-squares Cartesian velocity IK.
    """

    lambda2 = DAMPING * DAMPING

    A = (
        J @ J.T
        + lambda2 * np.eye(3, dtype=np.float64)
    )

    try:
        qdot = J.T @ np.linalg.solve(A, v)
    except np.linalg.LinAlgError:
        qdot = np.zeros(3, dtype=np.float64)

    return qdot


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class HotspotDirectJointVelocity(Node):

    def __init__(self):
        super().__init__("hotspot_direct_joint_velocity")

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

        # Target state
        self.target_visible = False
        self.err_x = 0.0
        self.err_y = 0.0
        self.conf = 0.0
        self.last_target_time = None

        # Joint state
        self.current_positions = {}
        self.have_all_joints = False

        # Centering state
        self.centered_x = False
        self.centered_y = False

        # Z reference state
        #
        # IMPORTANT:
        # hold_z is captured ONCE when valid tracking first begins.
        # It is not updated afterward.
        self.hold_z = None
        self.z_centered = True

        # Filters
        self.filtered_base_v = np.zeros(3, dtype=np.float64)
        self.filtered_qdot = np.zeros(3, dtype=np.float64)
        self.cmd_qdot = np.zeros(3, dtype=np.float64)

        self.lost_stop_publish_count = STOP_COMMANDS_AFTER_LOST

        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_command,
        )

        self.get_logger().info(
            "Hotspot direct joint velocity control started"
        )

        self.get_logger().info(
            f"Listening to {HOTSPOT_TOPIC}"
        )

        self.get_logger().info(
            f"Publishing direct joint velocities to {COMMAND_TOPIC}"
        )

        self.get_logger().warn(
            "MoveIt Servo is bypassed/removed."
        )

        self.get_logger().warn(
            "Z reference is captured when valid tracking starts."
        )

        self.get_logger().warn(
            "Z reference is fixed and is NOT accumulated or continuously updated."
        )

        self.get_logger().warn(
            f"Allowed Z deviation: +/-{Z_HOLD_DEADBAND * 100.0:.1f} cm"
        )

        self.get_logger().warn(
            f"XY reduction begins at {Z_SOFT_ERROR * 100.0:.1f} cm Z error"
        )

        self.get_logger().warn(
            f"XY stops at {Z_HARD_ERROR * 100.0:.1f} cm Z error"
        )

        self.get_logger().warn(
            "No per-joint velocity scaling after IK."
        )

        self.get_logger().warn(
            "No per-joint minimum velocity lifting."
        )

        self.get_logger().info(
            "Command order: [base_joint, shoulder_joint, elbow_joint]"
        )


    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

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

            for name, pos in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.current_positions[name] = float(pos)

            self.have_all_joints = all(
                joint in self.current_positions
                for joint in JOINT_NAMES
            )


    # -----------------------------------------------------------------------
    # State helpers
    # -----------------------------------------------------------------------

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

        return (
            time.monotonic() - self.last_target_time
        ) <= TARGET_TIMEOUT


    def capture_hold_z(self, tip):
        """
        Capture the fixed reference height.

        This is called only when valid tracking begins and hold_z has not
        previously been initialized.
        """

        if self.hold_z is not None:
            return

        self.hold_z = float(tip[2])
        self.z_centered = True

        self.get_logger().warn(
            f"Tracking Z reference captured: "
            f"hold_z={self.hold_z:+.4f} m"
        )


    # -----------------------------------------------------------------------
    # Stop / publication helpers
    # -----------------------------------------------------------------------

    def publish_zero_once(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]

        self.cmd_pub.publish(msg)


    def hard_stop(self):
        self.filtered_base_v[:] = 0.0
        self.filtered_qdot[:] = 0.0
        self.cmd_qdot[:] = 0.0

        self.publish_zero_once()


    # -----------------------------------------------------------------------
    # XY control
    # -----------------------------------------------------------------------

    def apply_hysteresis_deadband(self, err_x, err_y):

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


    def nonlinear_error_to_velocity(
        self,
        error,
        enter_deadband,
        max_vel,
        min_effective_vel,
    ):

        if error == 0.0:
            return 0.0

        magnitude = abs(error)

        usable_range = max(
            1e-6,
            ERROR_FULL_SPEED - enter_deadband,
        )

        normalized = clamp(
            (magnitude - enter_deadband) / usable_range,
            0.0,
            1.0,
        )

        # Quadratic shaping:
        # gentle around center, faster near image edges.
        shaped = normalized * normalized

        vel_abs = (
            min_effective_vel
            + (max_vel - min_effective_vel) * shaped
        )

        vel_abs = clamp(
            vel_abs,
            min_effective_vel,
            max_vel,
        )

        return math.copysign(
            vel_abs,
            error,
        )


    def compute_xy_base_velocity(
        self,
        active,
        err_x,
        err_y,
        conf,
    ):

        raw_v = np.zeros(
            3,
            dtype=np.float64,
        )

        if not active:
            self.centered_x = False
            self.centered_y = False
            return raw_v

        if conf < CONF_MIN:
            return raw_v

        err_x_used, err_y_used = (
            self.apply_hysteresis_deadband(
                err_x,
                err_y,
            )
        )

        vy_image = self.nonlinear_error_to_velocity(
            err_x_used,
            CENTER_ENTER_DEADBAND_X,
            MAX_CART_VEL_XY,
            MIN_EFFECTIVE_CART_VEL_XY,
        )

        vx_image = self.nonlinear_error_to_velocity(
            err_y_used,
            CENTER_ENTER_DEADBAND_Y,
            MAX_CART_VEL_XY,
            MIN_EFFECTIVE_CART_VEL_XY,
        )

        vx = (
            HOTSPOT_Y_SIGN_TO_BASE_X
            * vx_image
        )

        vy = (
            HOTSPOT_X_SIGN_TO_BASE_Y
            * vy_image
        )

        if not ENABLE_BASE_X:
            vx = 0.0

        if not ENABLE_BASE_Y:
            vy = 0.0

        raw_v[0] = clamp(
            vx,
            -MAX_CART_VEL_XY,
            MAX_CART_VEL_XY,
        )

        raw_v[1] = clamp(
            vy,
            -MAX_CART_VEL_XY,
            MAX_CART_VEL_XY,
        )

        raw_v[2] = 0.0

        return raw_v


    # -----------------------------------------------------------------------
    # Z control
    # -----------------------------------------------------------------------

    def compute_z_hold_velocity(self, tip):
        """
        Compute desired Z velocity relative to the ORIGINAL tracking-start
        height.

        +/-1 cm:
            No active Z correction.

        1-2 cm:
            Smoothly increasing correction.

        >=2 cm:
            Maximum Z correction.

        The reference itself never moves.
        """

        if not ENABLE_Z_HOLD:
            self.z_centered = True
            return 0.0

        if self.hold_z is None:
            self.z_centered = True
            return 0.0

        current_z = float(tip[2])

        # Absolute error relative to ORIGINAL reference.
        z_error = self.hold_z - current_z
        abs_error = abs(z_error)

        # Allowed region around original height.
        if abs_error <= Z_HOLD_DEADBAND:
            self.z_centered = True
            return 0.0

        self.z_centered = False

        correction_range = max(
            1e-6,
            Z_HOLD_HARD_BAND - Z_HOLD_DEADBAND,
        )

        normalized = clamp(
            (
                abs_error
                - Z_HOLD_DEADBAND
            )
            / correction_range,
            0.0,
            1.0,
        )

        # Smooth onset outside the allowed 1 cm region.
        shaped = normalized * normalized

        proportional_vz = (
            KZ_HOLD
            * z_error
        )

        vz = (
            proportional_vz
            * shaped
        )

        vz = clamp(
            vz,
            -MAX_Z_HOLD_VEL,
            MAX_Z_HOLD_VEL,
        )

        return vz


    def apply_z_priority_to_xy(
        self,
        raw_base_v,
        tip,
    ):
        """
        Reduce X/Y motion if the tool moves too far from the ORIGINAL height.

        <= 1.5 cm:
            Full XY.

        1.5-2.0 cm:
            XY scales continuously toward zero.

        >= 2.0 cm:
            XY fully stopped.
        """

        if self.hold_z is None:
            return raw_base_v

        z_error = (
            self.hold_z
            - float(tip[2])
        )

        abs_error = abs(z_error)

        if abs_error <= Z_SOFT_ERROR:
            return raw_base_v

        if abs_error >= Z_HARD_ERROR:
            raw_base_v[0] = 0.0
            raw_base_v[1] = 0.0

            return raw_base_v

        t = (
            abs_error
            - Z_SOFT_ERROR
        ) / (
            Z_HARD_ERROR
            - Z_SOFT_ERROR
        )

        xy_scale = clamp(
            1.0 - t,
            0.0,
            1.0,
        )

        raw_base_v[0] *= xy_scale
        raw_base_v[1] *= xy_scale

        return raw_base_v


    # -----------------------------------------------------------------------
    # Joint limits / qdot processing
    # -----------------------------------------------------------------------

    def apply_joint_limits(
        self,
        q,
        qdot,
    ):

        qdot_out = qdot.copy()

        for i, joint in enumerate(JOINT_NAMES):

            lower, upper = JOINT_LIMITS[joint]

            if (
                q[i] <= lower + JOINT_LIMIT_MARGIN
                and qdot_out[i] < 0.0
            ):

                self.get_logger().warn(
                    f"{joint} near lower limit: "
                    f"q={q[i]:+.3f}, blocking negative qdot",
                    throttle_duration_sec=1.0,
                )

                qdot_out[i] = 0.0

            if (
                q[i] >= upper - JOINT_LIMIT_MARGIN
                and qdot_out[i] > 0.0
            ):

                self.get_logger().warn(
                    f"{joint} near upper limit: "
                    f"q={q[i]:+.3f}, blocking positive qdot",
                    throttle_duration_sec=1.0,
                )

                qdot_out[i] = 0.0

        return qdot_out


    def postprocess_qdot(self, qdot):
        """
        Only operations that preserve the complete qdot direction should be
        applied whenever possible.

        Whole-vector scaling is allowed.
        Independent joint scaling is deliberately avoided.
        """

        out = qdot.copy()

        # Remove numerical noise.
        for i in range(3):
            if abs(out[i]) < JOINT_VEL_DEADBAND:
                out[i] = 0.0

        max_abs = float(
            np.max(
                np.abs(out)
            )
        )

        if max_abs <= 0.0:
            return out

        # Scale complete vector upward if necessary.
        # This preserves the joint ratio found by the IK.
        if USE_MIN_EFFECTIVE_QDOT_VECTOR:

            if max_abs < MIN_EFFECTIVE_QDOT_VECTOR:

                scale = (
                    MIN_EFFECTIVE_QDOT_VECTOR
                    / max_abs
                )

                out *= scale

        # Disabled by default because independent modification of joints
        # changes the Cartesian direction produced by the Jacobian.
        if USE_MIN_EFFECTIVE_QDOT_PER_JOINT:

            for i in range(3):

                if (
                    0.0
                    < abs(out[i])
                    < MIN_EFFECTIVE_QDOT_PER_JOINT
                ):
                    out[i] = math.copysign(
                        MIN_EFFECTIVE_QDOT_PER_JOINT,
                        out[i],
                    )

        max_abs = float(
            np.max(
                np.abs(out)
            )
        )

        # Maximum also uses complete-vector scaling.
        if max_abs > MAX_JOINT_VEL:

            out *= (
                MAX_JOINT_VEL
                / max_abs
            )

        return out


    # -----------------------------------------------------------------------
    # Main control loop
    # -----------------------------------------------------------------------

    def publish_command(self):

        q = self.get_q()

        if q is None:

            self.get_logger().warn(
                "ZERO CMD: missing joint states",
                throttle_duration_sec=1.0,
            )

            self.hard_stop()
            return

        tip = tip_position(q)

        with self.lock:
            target_visible = self.target_visible
            err_x = self.err_x
            err_y = self.err_y
            conf = self.conf
            fresh = self.target_is_fresh()

        active = (
            target_visible
            and fresh
            and conf >= CONF_MIN
        )

        # ---------------------------------------------------------------
        # Capture original Z reference ONLY when valid tracking starts.
        # ---------------------------------------------------------------

        if active and self.hold_z is None:
            self.capture_hold_z(tip)

        # ---------------------------------------------------------------
        # Target lost / invalid
        # ---------------------------------------------------------------

        if not active:

            reasons = []

            if not target_visible:
                reasons.append(
                    "target not visible"
                )

            if not fresh:
                reasons.append(
                    "target stale/timeout"
                )

            if conf < CONF_MIN:
                reasons.append(
                    f"confidence too low "
                    f"({conf:.3f} < {CONF_MIN:.3f})"
                )

            self.get_logger().warn(
                "ZERO CMD: "
                + ", ".join(reasons),
                throttle_duration_sec=1.0,
            )

            self.filtered_base_v[:] = 0.0
            self.filtered_qdot[:] = 0.0
            self.cmd_qdot[:] = 0.0

            self.centered_x = False
            self.centered_y = False

            if (
                self.lost_stop_publish_count
                < STOP_COMMANDS_AFTER_LOST
            ):

                self.publish_zero_once()

                self.lost_stop_publish_count += 1

            # IMPORTANT:
            # hold_z is deliberately NOT reset here.
            #
            # The original tracking-start height remains the reference
            # throughout the node lifetime.
            #
            # If you want a new reference after every target loss,
            # uncomment:
            #
            # self.hold_z = None

            return

        # ---------------------------------------------------------------
        # Cartesian velocity target
        # ---------------------------------------------------------------

        raw_base_v = (
            self.compute_xy_base_velocity(
                active,
                err_x,
                err_y,
                conf,
            )
        )

        # Explicit Z task:
        # This velocity is relative to the original reference height.
        raw_base_v[2] = (
            self.compute_z_hold_velocity(
                tip
            )
        )

        # If Z error becomes large, sacrifice XY tracking so the height can
        # recover.
        raw_base_v = (
            self.apply_z_priority_to_xy(
                raw_base_v,
                tip,
            )
        )

        # ---------------------------------------------------------------
        # Full centered condition
        # ---------------------------------------------------------------

        if (
            self.centered_x
            and self.centered_y
            and self.z_centered
        ):

            self.get_logger().info(
                "ZERO CMD: X/Y centered and Z inside allowed band",
                throttle_duration_sec=1.0,
            )

            self.hard_stop()
            return

        # ---------------------------------------------------------------
        # Cartesian low-pass filtering
        # ---------------------------------------------------------------

        self.filtered_base_v = (
            SMOOTHING_ALPHA
            * raw_base_v
            + (
                1.0
                - SMOOTHING_ALPHA
            )
            * self.filtered_base_v
        )

        self.filtered_base_v[0] = clamp(
            self.filtered_base_v[0],
            -MAX_CART_VEL_XY,
            MAX_CART_VEL_XY,
        )

        self.filtered_base_v[1] = clamp(
            self.filtered_base_v[1],
            -MAX_CART_VEL_XY,
            MAX_CART_VEL_XY,
        )

        self.filtered_base_v[2] = clamp(
            self.filtered_base_v[2],
            -MAX_Z_HOLD_VEL,
            MAX_Z_HOLD_VEL,
        )

        if (
            float(
                np.linalg.norm(
                    self.filtered_base_v
                )
            )
            <= COMMAND_EPSILON
        ):

            self.get_logger().warn(
                "ZERO CMD: filtered Cartesian velocity below epsilon "
                f"v={self.filtered_base_v}",
                throttle_duration_sec=1.0,
            )

            self.hard_stop()
            return

        # ---------------------------------------------------------------
        # Jacobian / IK
        # ---------------------------------------------------------------

        J = numeric_position_jacobian(q)

        qdot_raw = damped_least_squares(
            J,
            self.filtered_base_v,
        )

        # ---------------------------------------------------------------
        # Joint limits
        # ---------------------------------------------------------------

        qdot_limited = self.apply_joint_limits(
            q,
            qdot_raw,
        )

        # IMPORTANT:
        #
        # Do NOT do:
        #
        #   shoulder *= 2.2
        #   elbow *= 0.9
        #
        # or similar here.
        #
        # That would destroy the joint ratio chosen by the IK and therefore
        # produce unintended Cartesian Z velocity.

        qdot_limited = (
            self.postprocess_qdot(
                qdot_limited
            )
        )

        # ---------------------------------------------------------------
        # Joint velocity smoothing
        # ---------------------------------------------------------------

        self.filtered_qdot = (
            JOINT_SMOOTHING_ALPHA
            * qdot_limited
            + (
                1.0
                - JOINT_SMOOTHING_ALPHA
            )
            * self.filtered_qdot
        )

        # Whole-vector limits / minimum again after filtering.
        self.filtered_qdot = (
            self.postprocess_qdot(
                self.filtered_qdot
            )
        )

        self.cmd_qdot = (
            self.filtered_qdot.copy()
        )

        # ---------------------------------------------------------------
        # Diagnostic: what Cartesian velocity will this qdot theoretically
        # produce according to the same Jacobian?
        # ---------------------------------------------------------------

        predicted_cart_v = (
            J @ self.cmd_qdot
        )

        if self.hold_z is not None:

            z_error = (
                self.hold_z
                - float(tip[2])
            )

        else:
            z_error = 0.0

        self.get_logger().info(
            "CTRL "
            f"z={tip[2]:+.4f} "
            f"hold_z={self.hold_z if self.hold_z is not None else 0.0:+.4f} "
            f"z_err={z_error:+.4f} | "
            f"v_req=["
            f"{self.filtered_base_v[0]:+.5f}, "
            f"{self.filtered_base_v[1]:+.5f}, "
            f"{self.filtered_base_v[2]:+.5f}] | "
            f"v_pred=["
            f"{predicted_cart_v[0]:+.5f}, "
            f"{predicted_cart_v[1]:+.5f}, "
            f"{predicted_cart_v[2]:+.5f}]",
            throttle_duration_sec=0.5,
        )

        # ---------------------------------------------------------------
        # Final zero test
        # ---------------------------------------------------------------

        if (
            abs(self.cmd_qdot[0]) < COMMAND_EPSILON
            and abs(self.cmd_qdot[1]) < COMMAND_EPSILON
            and abs(self.cmd_qdot[2]) < COMMAND_EPSILON
        ):

            self.get_logger().warn(
                "ZERO CMD: final joint velocity below epsilon "
                f"qdot={self.cmd_qdot}",
                throttle_duration_sec=1.0,
            )

            self.hard_stop()
            return

        # ---------------------------------------------------------------
        # Publish joint velocities
        # ---------------------------------------------------------------

        msg = Float64MultiArray()

        msg.data = [
            float(self.cmd_qdot[0]),
            float(self.cmd_qdot[1]),
            float(self.cmd_qdot[2]),
        ]

        self.cmd_pub.publish(msg)

        self.lost_stop_publish_count = 0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = HotspotDirectJointVelocity()

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
