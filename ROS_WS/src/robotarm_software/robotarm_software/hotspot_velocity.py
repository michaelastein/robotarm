#!/usr/bin/env python3
"""
Direct joint-velocity controller for centering a detected image hotspot.

The node receives normalized image-space hotspot errors, converts them into a
small Cartesian tool-tip velocity, and maps that velocity to joint velocities
with a numerically calculated Jacobian and damped least-squares inverse
kinematics. A slow Z-height correction keeps the tool tip near the height at
which tracking began.

Configuration overview:
JOINT_NAMES:
Ordered joint names used for state lookup, limit enforcement, and
command publication. The published velocity vector always follows this
order: base, shoulder, elbow.

HOTSPOT_TOPIC:
    Float32MultiArray input topic. Expected fields are visible flag,
    horizontal image error, vertical image error, and detector confidence.

COMMAND_TOPIC:
    Float64MultiArray output topic carrying direct joint velocities in
    radians per second.

PUBLISH_PERIOD:
    Timer period in seconds. A value of 0.01 runs the control loop at 100 Hz.

CONF_MIN / TARGET_TIMEOUT:
    Minimum accepted detector confidence and maximum age of a hotspot
    message before it is treated as stale.

CENTER_*_DEADBAND_*:
    Enter/exit hysteresis thresholds for image X and Y errors. Separate
    thresholds prevent rapid switching and circular motion near center.

MAX_CART_VEL_FAR / MAX_CART_VEL_NEAR / MIN_EFFECTIVE_CART_VEL_XY:
    Adaptive maximum Cartesian speed in the base X/Y plane. The controller
    allows a higher maximum speed when far from the target and reduces the
    maximum speed near the target. The minimum helps overcome hardware deadband.

HOTSPOT_*_SIGN_TO_BASE_*:
    Sign mappings from camera-image error directions to base-frame motion.

Z_HOLD_* / KZ_HOLD / MAX_Z_HOLD_VEL:
    Parameters for slow tool-tip height correction. Z correction is
    deliberately weaker than hotspot-centering motion.

Z_SOFT_ERROR / Z_HARD_ERROR:
    Height-error thresholds that reduce or stop X/Y motion so Z can recover.

SMOOTHING_ALPHA / JOINT_SMOOTHING_ALPHA:
    Low-pass filter weights for Cartesian and joint-velocity commands.
    Larger values react faster; smaller values produce smoother motion.

DAMPING / JACOBIAN_EPS:
    Damped least-squares regularization and finite-difference step used for
    the numerical position Jacobian.

MAX_JOINT_VEL / MIN_EFFECTIVE_QDOT_VECTOR:
    Maximum joint speed and optional whole-vector minimum. Scaling the full
    vector preserves the inverse-kinematics joint ratio.

JOINT_LIMITS / JOINT_LIMIT_MARGIN:
    Per-joint position bounds and the safety margin inside each bound where
    commands farther into the limit are blocked.

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

# Joints / topics

JOINT_NAMES = [
"base_joint",
"shoulder_joint",
"elbow_joint",
]

HOTSPOT_TOPIC = "/hotspot/target"
COMMAND_TOPIC = "/velocity_controller/commands"

PUBLISH_PERIOD = 0.01  # 100 Hz

# Hotspot input format

#

# /hotspot/target:

# data[0] = visible, >= 0.5 means visible

# data[1] = err_x

# data[2] = err_y

# data[3] = confidence

#

# Mapping:

#

# hotspot left:

# err_x negative -> move arm left  -> +base_link y

#

# hotspot right:

# err_x positive -> move arm right -> -base_link y

#

# hotspot lower:

# err_y positive -> move arm back  -> -base_link x

#

# hotspot upper:

# err_y negative -> move arm front -> +base_link x

# Hotspot centering control

CONF_MIN = 0.0
TARGET_TIMEOUT = 0.25

# Larger hysteresis against circling around a stationary LED.

CENTER_ENTER_DEADBAND_X = 0.15
CENTER_EXIT_DEADBAND_X = 0.15

CENTER_ENTER_DEADBAND_Y = 0.15
CENTER_EXIT_DEADBAND_Y = 0.15

MAX_CART_VEL_FAR = 0.01
MAX_CART_VEL_NEAR = 0.003
MIN_EFFECTIVE_CART_VEL_XY = 0.0015

# Error range used to interpolate the adaptive Cartesian maximum speed.
# At/below ADAPTIVE_VEL_ERROR_NEAR -> MAX_CART_VEL_NEAR.
# At/above ADAPTIVE_VEL_ERROR_FAR  -> MAX_CART_VEL_FAR.
ADAPTIVE_VEL_ERROR_NEAR = 0.10
ADAPTIVE_VEL_ERROR_FAR = 0.70

ERROR_FULL_SPEED = 0.7

HOTSPOT_X_SIGN_TO_BASE_Y = -1.0
HOTSPOT_Y_SIGN_TO_BASE_X = -1.0

ENABLE_BASE_X = True
ENABLE_BASE_Y = True

# Z height hold

#

# Z is only a slow drift correction.

# It should not constantly fight tiny height changes.

ENABLE_Z_HOLD = True

# More relaxed Z hysteresis.

Z_HOLD_DEADBAND = 0.03
Z_HOLD_EXIT_DEADBAND = 0.04

KZ_HOLD = 0.006
MAX_Z_HOLD_VEL = 0.00035
MIN_EFFECTIVE_Z_VEL = 0.0000

Z_HOLD_SCALE_WHILE_XY_MOVING = 1.00

# If Z drifts far, slow/stop XY so height can recover.

Z_SOFT_ERROR = 0.055
Z_HARD_ERROR = 0.080
XY_SCALE_WHEN_Z_SOFT_ERROR = 0.25

# Filtering / stopping

SMOOTHING_ALPHA = 0.18
LOST_ALPHA = 0.35

STOP_COMMANDS_AFTER_LOST = 10

COMMAND_EPSILON = 0.00001

# IK / Jacobian parameters

DAMPING = 0.035
JACOBIAN_EPS = 1e-4

# Joint velocity output

#

# Hardware deadband is around 0.003 rad/s.

# If Python sends tiny qdot values like 0.0003, nothing happens.

#

# Therefore use a small VECTOR minimum.

# This preserves the IK ratio, unlike per-joint minimum lifting.

MAX_JOINT_VEL = 0.20

USE_MIN_EFFECTIVE_QDOT_VECTOR = False
MIN_EFFECTIVE_QDOT_VECTOR = 0.03

# Controller-side startup floor.
# The hardware interface ignores |qdot| <= 0.003 rad/s, so intended
# non-zero joint commands are lifted only slightly above that threshold.
USE_MIN_EFFECTIVE_QDOT_PER_JOINT = True
MIN_EFFECTIVE_QDOT_PER_JOINT = 0.0035

JOINT_VEL_DEADBAND = 0.00005
JOINT_SMOOTHING_ALPHA = 0.20

# Per-joint output scaling.
# Keep all IK joint ratios unchanged. Cartesian velocity is the primary
# speed-control mechanism; joint limits and MAX_JOINT_VEL remain safety caps.
BASE_JOINT_VEL_SCALE = 1.0
SHOULDER_JOINT_VEL_SCALE = 1.0

# Joint limits

JOINT_LIMITS = {
"base_joint": (-3.0, 3.0),
"shoulder_joint": (-0.52359878, 1.39626340),
"elbow_joint": (-0.69813170, 2.44346095),
}

JOINT_LIMIT_MARGIN = 0.05

def clamp(value, low, high):
    """
    Restrict a numeric value to an inclusive range.

    Parameters:
        value: Number to constrain.
        low: Smallest permitted result.
        high: Largest permitted result.

    Returns:
        The input value clipped to the interval [low, high].
    """
    return max(low, min(high, value))

def rot_z(theta):
    """
    Build a homogeneous rotation matrix about the Z axis.

    Parameters:
        theta: Rotation angle in radians.

    Returns:
        A 4x4 NumPy homogeneous transformation matrix using float64 values.
    """
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
    """
    Build a homogeneous rotation matrix about the Y axis.

    Parameters:
        theta: Rotation angle in radians.

    Returns:
        A 4x4 NumPy homogeneous transformation matrix using float64 values.
    """
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
    """
    Build a homogeneous translation matrix.

    Parameters:
        x: Translation along the X axis in meters.
        y: Translation along the Y axis in meters.
        z: Translation along the Z axis in meters.

    Returns:
        A 4x4 NumPy homogeneous transformation matrix using float64 values.
    """
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
    Compute the tool-tip pose relative to base_link.

    Parameters:
        q: Three-element joint-position array in radians, ordered as
            [base_joint, shoulder_joint, elbow_joint].

    Returns:
        A 4x4 homogeneous transformation matrix from base_link to
        tool_tip_link.
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
    """
    Compute only the Cartesian tool-tip position.

    Parameters:
        q: Three-element joint-position array in radians, ordered as
            [base_joint, shoulder_joint, elbow_joint].

    Returns:
        A length-three float64 array containing [x, y, z] in meters.
    """
    T = forward_kinematics(q)
    return T[0:3, 3].copy()

def numeric_position_jacobian(q):
    """
    Estimate the translational Jacobian with forward finite differences.

    Parameters:
        q: Three-element joint-position array in radians.

    Returns:
        A 3x3 matrix mapping joint velocity to tool-tip linear velocity.
        Rows correspond to base-frame X, Y, and Z; columns follow JOINT_NAMES.
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
    Convert a Cartesian velocity into joint velocities using damped IK.

    Parameters:
        J: 3x3 translational Jacobian.
        v: Desired three-element Cartesian tool-tip velocity in meters per
            second, expressed in base_link.

    Returns:
        A three-element joint-velocity vector in radians per second. A zero
        vector is returned if the linear system cannot be solved.
    """
    lambda2 = DAMPING * DAMPING
    A = J @ J.T + lambda2 * np.eye(3, dtype=np.float64)

    try:
        qdot = J.T @ np.linalg.solve(A, v)
    except np.linalg.LinAlgError:
        qdot = np.zeros(3, dtype=np.float64)

    return qdot

class HotspotDirectJointVelocity(Node):

    def __init__(self):
        """
        Initialize ROS interfaces, controller state, filters, and diagnostics.

        Parameters:
            None. ROS arguments are handled by rclpy before node construction.

        Returns:
            None.
        """
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

        self.target_visible = False
        self.err_x = 0.0
        self.err_y = 0.0
        self.conf = 0.0
        self.last_target_time = None

        self.current_positions = {}
        self.have_all_joints = False

        self.centered_x = False
        self.centered_y = False
        self.z_centered = True

        self.hold_z = None

        self.filtered_base_v = np.zeros(3, dtype=np.float64)
        self.filtered_qdot = np.zeros(3, dtype=np.float64)
        self.cmd_qdot = np.zeros(3, dtype=np.float64)

        self.lost_stop_publish_count = STOP_COMMANDS_AFTER_LOST


        self.timer = self.create_timer(
            PUBLISH_PERIOD,
            self.publish_command,
        )

        self.get_logger().info("Hotspot direct joint velocity control started")
        self.get_logger().info(f"Listening to {HOTSPOT_TOPIC}")
        self.get_logger().info(f"Publishing direct joint velocities to {COMMAND_TOPIC}")
        self.get_logger().warn("MoveIt Servo is bypassed/removed.")
        self.get_logger().warn("Centering controller active.")
        self.get_logger().warn(
            f"Adaptive XY max velocity: near={MAX_CART_VEL_NEAR:.4f} m/s "
            f"at error<={ADAPTIVE_VEL_ERROR_NEAR:.2f}, "
            f"far={MAX_CART_VEL_FAR:.4f} m/s "
            f"at error>={ADAPTIVE_VEL_ERROR_FAR:.2f}"
        )
        self.get_logger().warn("Z-hold is slow drift correction only.")
        self.get_logger().warn("No pulse mode.")
        self.get_logger().warn(
            "Whole-vector qdot minimum disabled; Cartesian velocity drives speed."
        )
        self.get_logger().warn(
            "Only the small per-joint hardware-deadband floor remains active."
        )
        self.get_logger().warn(
            f"XY hysteresis: x enter={CENTER_ENTER_DEADBAND_X:.2f}, "
            f"x exit={CENTER_EXIT_DEADBAND_X:.2f}, "
            f"y enter={CENTER_ENTER_DEADBAND_Y:.2f}, "
            f"y exit={CENTER_EXIT_DEADBAND_Y:.2f}"
        )
        self.get_logger().warn(
            f"Z hysteresis: enter={Z_HOLD_DEADBAND:.3f} m, "
            f"exit={Z_HOLD_EXIT_DEADBAND:.3f} m"
        )
        self.get_logger().warn(
            f"qdot vector minimum: enabled={USE_MIN_EFFECTIVE_QDOT_VECTOR}, "
            f"min={MIN_EFFECTIVE_QDOT_VECTOR:.4f}, max={MAX_JOINT_VEL:.4f}"
        )
        self.get_logger().info("Command order is fixed: [base_joint, shoulder_joint, elbow_joint]")
        self.get_logger().info("Mapping: err_x negative -> +base_y left")
        self.get_logger().info("Mapping: err_y positive -> -base_x back")

    # Callbacks

    def target_callback(self, msg):
        """
        Store the latest hotspot detection message.

        Parameters:
            msg: Float32MultiArray with at least four entries:
                data[0] visible flag, data[1] horizontal error,
                data[2] vertical error, and data[3] confidence.

        Returns:
            None. Messages shorter than four values are ignored.
        """
        if len(msg.data) < 4:
            return

        with self.lock:
            self.target_visible = msg.data[0] >= 0.5
            self.err_x = float(msg.data[1])
            self.err_y = float(msg.data[2])
            self.conf = float(msg.data[3])
            self.last_target_time = time.monotonic()

    def joint_state_callback(self, msg):
        """
        Update known joint positions from a JointState message.

        Parameters:
            msg: JointState message. Positions are matched by joint name rather
                than by array index, so unrelated joints and ordering changes
                are tolerated.

        Returns:
            None. The internal ready flag becomes true after all required joint
            positions have been received.
        """
        with self.lock:

            for name, pos in zip(msg.name, msg.position):
                if name in JOINT_NAMES:
                    self.current_positions[name] = float(pos)

            self.have_all_joints = all(
                joint in self.current_positions
                for joint in JOINT_NAMES
            )

    # State helpers

    def get_q(self):
        """
        Return the current required joint positions in command order.

        Parameters:
            None.

        Returns:
            A float64 array ordered as JOINT_NAMES, or None until all required
            joint positions have been received.
        """
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
        """
        Check whether the latest hotspot message is recent enough for control.

        Parameters:
            None.

        Returns:
            True when a target timestamp exists and its age does not exceed
            TARGET_TIMEOUT; otherwise False.
        """
        if self.last_target_time is None:
            return False

        return (time.monotonic() - self.last_target_time) <= TARGET_TIMEOUT

    def reset_hold_z_if_needed(self, tip):
        """
        Initialize the Z-height reference from the current tool-tip position.

        Parameters:
            tip: Three-element tool-tip position [x, y, z] in meters.

        Returns:
            None. Initialization occurs only once unless hold_z is reset
            elsewhere.
        """
        if self.hold_z is None:
            self.hold_z = float(tip[2])
            self.z_centered = True
            self.get_logger().warn(
                f"Z hold initialized: hold_z={self.hold_z:+.4f} m"
            )

    # Command helpers

    def publish_zero_once(self):
        """
        Publish one zero joint-velocity command.

        Parameters:
            None.

        Returns:
            None.
        """
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.cmd_pub.publish(msg)

    def hard_stop(self):
        """
        Clear all velocity filters and immediately publish a zero command.

        Parameters:
            None.

        Returns:
            None.
        """
        self.filtered_base_v[:] = 0.0
        self.filtered_qdot[:] = 0.0
        self.cmd_qdot[:] = 0.0
        self.publish_zero_once()

    def apply_hysteresis_deadband(self, err_x, err_y):
        """
        Apply independent enter/exit deadbands to hotspot X and Y errors.

        Parameters:
            err_x: Normalized horizontal image error.
            err_y: Normalized vertical image error.

        Returns:
            A tuple (err_x_used, err_y_used). An axis is returned as zero while
            its centered state remains inside the hysteresis region.
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

    def nonlinear_error_to_velocity(
        self,
        error,
        enter_deadband,
        max_vel,
        min_effective_vel,
    ):
        """
        Map one signed normalized error to a shaped Cartesian speed.

        Parameters:
            error: Signed image-space error after deadband handling.
            enter_deadband: Error magnitude at which useful motion begins.
            max_vel: Maximum output speed magnitude.
            min_effective_vel: Minimum nonzero output speed magnitude.

        Returns:
            A signed velocity. Magnitude grows quadratically from the minimum
            effective speed to max_vel as the error approaches full scale.
        """
        if error == 0.0:
            return 0.0

        a = abs(error)

        usable_range = max(1e-6, ERROR_FULL_SPEED - enter_deadband)
        normalized = clamp((a - enter_deadband) / usable_range, 0.0, 1.0)

        # Quadratic: soft near center, fast near edge.
        shaped = normalized * normalized

        vel_abs = min_effective_vel + (
            max_vel - min_effective_vel
        ) * shaped

        vel_abs = clamp(vel_abs, min_effective_vel, max_vel)

        return math.copysign(vel_abs, error)

    def compute_xy_base_velocity(self, active, err_x, err_y, conf):
        """
        Convert hotspot errors into a base-frame X/Y velocity command.

        The maximum Cartesian X/Y speed is adaptive:
        - far from the target: up to MAX_CART_VEL_FAR
        - near the target:    down to MAX_CART_VEL_NEAR

        The interpolation is based on the larger absolute deadband-filtered
        image error so both Cartesian axes share the same speed envelope.

        Parameters:
            active: Whether target tracking is currently permitted.
            err_x: Normalized horizontal image error.
            err_y: Normalized vertical image error.
            conf: Detector confidence associated with the errors.

        Returns:
            A three-element float64 velocity vector [vx, vy, 0] in meters per
            second, expressed in base_link.
        """
        raw_v = np.zeros(3, dtype=np.float64)

        if not active:
            self.centered_x = False
            self.centered_y = False
            return raw_v

        if conf < CONF_MIN:
            return raw_v

        err_x_used, err_y_used = self.apply_hysteresis_deadband(
            err_x,
            err_y,
        )

        error_mag = max(
            abs(err_x_used),
            abs(err_y_used),
        )

        adaptive_denominator = max(
            1e-6,
            ADAPTIVE_VEL_ERROR_FAR - ADAPTIVE_VEL_ERROR_NEAR,
        )

        scale = clamp(
            (error_mag - ADAPTIVE_VEL_ERROR_NEAR) / adaptive_denominator,
            0.0,
            1.0,
        )

        max_cart_vel = (
            MAX_CART_VEL_NEAR
            + scale * (MAX_CART_VEL_FAR - MAX_CART_VEL_NEAR)
        )

        vy_image = self.nonlinear_error_to_velocity(
            err_x_used,
            CENTER_ENTER_DEADBAND_X,
            max_cart_vel,
            MIN_EFFECTIVE_CART_VEL_XY,
        )

        vx_image = self.nonlinear_error_to_velocity(
            err_y_used,
            CENTER_ENTER_DEADBAND_Y,
            max_cart_vel,
            MIN_EFFECTIVE_CART_VEL_XY,
        )

        vx = HOTSPOT_Y_SIGN_TO_BASE_X * vx_image
        vy = HOTSPOT_X_SIGN_TO_BASE_Y * vy_image

        if not ENABLE_BASE_X:
            vx = 0.0

        if not ENABLE_BASE_Y:
            vy = 0.0

        raw_v[0] = clamp(vx, -max_cart_vel, max_cart_vel)
        raw_v[1] = clamp(vy, -max_cart_vel, max_cart_vel)
        raw_v[2] = 0.0

        return raw_v

    def compute_z_hold_velocity(self, tip, xy_is_moving):
        """
        Compute the slow Z velocity used to maintain the stored height.

        Parameters:
            tip: Current three-element tool-tip position in meters.
            xy_is_moving: Whether an X/Y correction is currently requested.
                Retained for controller tuning and future scaling logic.

        Returns:
            Signed base-frame Z velocity in meters per second, or zero when Z
            hold is disabled, uninitialized, or inside its hysteresis band.
        """
        if not ENABLE_Z_HOLD:
            return 0.0

        if self.hold_z is None:
            return 0.0

        z_error = self.hold_z - float(tip[2])
        abs_z_error = abs(z_error)

        # Z hysteresis:
        # If already centered, stay quiet until error is clearly large.
        # If correcting, stop after returning into the smaller band.
        if self.z_centered:
            if abs_z_error > Z_HOLD_EXIT_DEADBAND:
                self.z_centered = False
            else:
                return 0.0
        else:
            if abs_z_error < Z_HOLD_DEADBAND:
                self.z_centered = True
                return 0.0

        vz = KZ_HOLD * z_error
        vz = clamp(vz, -MAX_Z_HOLD_VEL, MAX_Z_HOLD_VEL)

        # No minimum Z velocity. Minimum caused Z oscillation.
        return vz

    def apply_z_priority_to_xy(self, raw_base_v, tip):
        """
        Reduce X/Y motion when the tool-tip height error is excessive.

        Parameters:
            raw_base_v: Mutable three-element Cartesian velocity vector.
            tip: Current three-element tool-tip position in meters.

        Returns:
            The adjusted velocity vector. X/Y are scaled at the soft threshold
            and set to zero at the hard threshold.
        """
        if self.hold_z is None:
            return raw_base_v

        z_error = self.hold_z - float(tip[2])
        z_error_abs = abs(z_error)

        if z_error_abs > Z_HARD_ERROR:
            raw_base_v[0] = 0.0
            raw_base_v[1] = 0.0
            return raw_base_v

        if z_error_abs > Z_SOFT_ERROR:
            raw_base_v[0] *= XY_SCALE_WHEN_Z_SOFT_ERROR
            raw_base_v[1] *= XY_SCALE_WHEN_Z_SOFT_ERROR
            return raw_base_v

        return raw_base_v

    def apply_joint_limits(self, q, qdot):
        """
        Block joint velocities that would move farther into a position limit.

        Parameters:
            q: Current three-element joint-position vector in radians.
            qdot: Requested three-element joint-velocity vector in radians per
                second.

        Returns:
            A copied and limit-safe joint-velocity vector.
        """
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
        Apply joint deadband, optional hardware-deadband handling, and safety
        speed limiting.

        Parameters:
            qdot: Three-element joint-velocity vector in radians per second.

        Returns:
            A processed copy. With whole-vector minimum scaling disabled,
            inverse-kinematics magnitudes are left unchanged unless a command
            must clear the hardware deadband or exceeds MAX_JOINT_VEL.
        """

        out = qdot.copy()

        for i in range(3):
            if abs(out[i]) < JOINT_VEL_DEADBAND:
                out[i] = 0.0

        max_abs = float(np.max(np.abs(out)))

        if max_abs <= 0.0:
            return out

        if USE_MIN_EFFECTIVE_QDOT_VECTOR:
            if max_abs < MIN_EFFECTIVE_QDOT_VECTOR:
                out *= MIN_EFFECTIVE_QDOT_VECTOR / max_abs

        # Only remaining intentional qdot modification below the safety cap:
        # lift tiny intended non-zero commands just above the hardware velocity
        # deadband. Zero commands remain zero and signs are preserved.
        if USE_MIN_EFFECTIVE_QDOT_PER_JOINT:
            for i in range(3):
                if 0.0 < abs(out[i]) < MIN_EFFECTIVE_QDOT_PER_JOINT:
                    out[i] = math.copysign(MIN_EFFECTIVE_QDOT_PER_JOINT, out[i])

        max_abs = float(np.max(np.abs(out)))

        if max_abs > MAX_JOINT_VEL:
            out *= MAX_JOINT_VEL / max_abs

        return out


    # Main control loop

    def publish_command(self):
        """
        Run one complete control cycle and publish a joint-velocity command.

        Parameters:
            None. The method reads the latest joint and hotspot state stored by
            the subscriber callbacks.

        Returns:
            None. Depending on state, it publishes a tracking command, a finite
            sequence of zero stop commands, or no command.
        """
        q = self.get_q()

        if q is None:
            self.get_logger().warn(
                "ZERO CMD: missing joint states",
                throttle_duration_sec=1.0,
            )
            self.hard_stop()
            return

        tip = tip_position(q)
        self.reset_hold_z_if_needed(tip)

        with self.lock:
            target_visible = self.target_visible
            err_x = self.err_x
            err_y = self.err_y
            conf = self.conf
            fresh = self.target_is_fresh()

        active = target_visible and fresh and conf >= CONF_MIN

        J = numeric_position_jacobian(q)

        if not active:
            reasons = []
            if not target_visible:
                reasons.append("target not visible")
            if not fresh:
                reasons.append("target stale/timeout")
            if conf < CONF_MIN:
                reasons.append(
                    f"confidence too low ({conf:.3f} < {CONF_MIN:.3f})"
                )

            self.get_logger().warn(
                "ZERO CMD: " + ", ".join(reasons),
                throttle_duration_sec=1.0,
            )

            self.filtered_base_v[:] = 0.0
            self.filtered_qdot[:] = 0.0
            self.cmd_qdot[:] = 0.0
            self.centered_x = False
            self.centered_y = False

            if self.lost_stop_publish_count < STOP_COMMANDS_AFTER_LOST:
                self.publish_zero_once()
                self.lost_stop_publish_count += 1

            return

        raw_base_v = self.compute_xy_base_velocity(
            active,
            err_x,
            err_y,
            conf,
        )

        xy_is_moving = (
            abs(raw_base_v[0]) > 0.0
            or abs(raw_base_v[1]) > 0.0
        )

        raw_base_v[2] = self.compute_z_hold_velocity(
            tip,
            xy_is_moving,
        )

        raw_base_v = self.apply_z_priority_to_xy(
            raw_base_v,
            tip,
        )

        # Critical anti-circling stop:
        # Only hard-stop when image X, image Y, and Z are all centered.
        # Do NOT hard-stop just because qdot is tiny while the target is still off-center.
        if self.centered_x and self.centered_y and self.z_centered:
            self.get_logger().info(
                "ZERO CMD: target centered in X/Y and Z hold centered",
                throttle_duration_sec=1.0,
            )
            self.hard_stop()
            return

        alpha = SMOOTHING_ALPHA if active else LOST_ALPHA

        self.filtered_base_v = (
            alpha * raw_base_v
            + (1.0 - alpha) * self.filtered_base_v
        )

        self.filtered_base_v[0] = clamp(
            self.filtered_base_v[0],
            -MAX_CART_VEL_FAR,
            MAX_CART_VEL_FAR,
        )

        self.filtered_base_v[1] = clamp(
            self.filtered_base_v[1],
            -MAX_CART_VEL_FAR,
            MAX_CART_VEL_FAR,
        )

        self.filtered_base_v[2] = clamp(
            self.filtered_base_v[2],
            -MAX_Z_HOLD_VEL,
            MAX_Z_HOLD_VEL,
        )

        if float(np.linalg.norm(self.filtered_base_v)) <= COMMAND_EPSILON:
            self.get_logger().warn(
                "ZERO CMD: filtered Cartesian velocity below epsilon "
                f"v={self.filtered_base_v}, eps={COMMAND_EPSILON}",
                throttle_duration_sec=1.0,
            )
            return

        qdot_raw = damped_least_squares(
            J,
            self.filtered_base_v,
        )

        qdot_limited = self.apply_joint_limits(q, qdot_raw)

        # Keep IK joint ratios unchanged. These scales are intentionally 1.0;
        # Cartesian velocity is the primary speed-control mechanism.
        qdot_limited[0] *= BASE_JOINT_VEL_SCALE
        qdot_limited[1] *= SHOULDER_JOINT_VEL_SCALE

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
            self.get_logger().warn(
                "ZERO CMD: final joint velocity below epsilon "
                f"qdot={self.cmd_qdot}, eps={COMMAND_EPSILON}",
                throttle_duration_sec=1.0,
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

def main(args=None):
    """
    Initialize ROS, run the velocity-control node, and perform a safe shutdown.

    Parameters:
        args: Optional ROS argument sequence passed to rclpy.init().

    Returns:
        None. A final zero command is published during normal shutdown when the
        ROS context is still active.
    """
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
