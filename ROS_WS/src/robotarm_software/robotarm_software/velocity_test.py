#!/usr/bin/env python3

"""
Automatic minimum-motion test for a ROS 2 velocity-controlled robot arm.

What it does:
- Tests one joint at a time.
- Tests positive and negative direction separately.
- Increases the commanded joint velocity step by step.
- Watches /joint_states for real encoder movement.
- Reports:
    * minimum command velocity that caused movement
    * measured velocity
    * estimated PWM from the current hardware formula

Important:
- This does NOT command raw PWM directly.
- The PWM value is estimated using the same formula currently used in
  RobotArmSystem::write().
- Stop the safety_supervisor or ensure it does not overwrite commands.
- Keep the emergency stop accessible.
- Start with the arm away from mechanical limits.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

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
JOINT_STATE_TOPIC = "/joint_states"

PUBLISH_RATE_HZ = 50.0
COMMAND_PERIOD = 1.0 / PUBLISH_RATE_HZ

WAIT_FOR_JOINT_STATES_S = 5.0
JOINT_STATE_STALE_TIMEOUT_S = 0.5

# Each test step is held for this long.
STEP_DURATION_S = 1.2

# Pause after every test step.
SETTLE_TIME_S = 0.5

# Stop after detected motion and gather a little extra data.
CONFIRM_MOTION_TIME_S = 0.35

# Encoder movement required to count as real motion.
# Increase this if encoder noise causes false positives.
MIN_POSITION_CHANGE_RAD = {
    "base_joint": 0.012,
    "shoulder_joint": 0.008,
    "elbow_joint": 0.008,
}

# Reject apparent movement that is only a single noisy sample.
MIN_MOVING_SAMPLES = 3

# Stop a direction test before getting too close to the limits.
LIMIT_MARGIN_RAD = {
    "base_joint": 0.20,
    "shoulder_joint": 0.15,
    "elbow_joint": 0.15,
}

JOINT_LIMITS = {
    "base_joint": (-3.14159265, 3.14159265),
    "shoulder_joint": (-0.52359878, 1.39626340),
    "elbow_joint": (-0.69813170, 2.44346095),
}

# Velocity commands to test.
# These start low and rise gradually.
TEST_VELOCITIES = {
    "base_joint": [
        0.006,
        0.008,
        0.010,
        0.015,
        0.020,
        0.030,
        0.040,
        0.060,
        0.080,
        0.100,
        0.150,
        0.200,
    ],
    "shoulder_joint": [
        0.050,
        0.075,
        0.100,
        0.125,
        0.150,
        0.200,
        0.250,
        0.300,
        0.400,
    ],
    "elbow_joint": [
        0.050,
        0.075,
        0.100,
        0.125,
        0.150,
        0.200,
        0.250,
        0.300,
        0.400,
    ],
}

# Current hardware-plugin parameters.
# Keep these synchronized with RobotArmSystem::on_init().
MIN_PWM = {
    "base_joint": 0.008,
    "shoulder_joint": 0.025,
    "elbow_joint": 0.025,
}

MAX_PWM = {
    "base_joint": 0.20,
    "shoulder_joint": 0.50,
    "elbow_joint": 0.50,
}

MIN_COMMAND_VELOCITY = {
    "base_joint": 0.006,
    "shoulder_joint": 0.10,
    "elbow_joint": 0.10,
}

MAX_JOINT_VELOCITY = {
    "base_joint": 0.30,
    "shoulder_joint": 0.60,
    "elbow_joint": 0.60,
}

VELOCITY_TO_PWM_GAIN = {
    "base_joint": 0.006,
    "shoulder_joint": 0.001,
    "elbow_joint": 0.001,
}

VELOCITY_KP = {
    "base_joint": 0.002,
    "shoulder_joint": 0.001,
    "elbow_joint": 0.001,
}


@dataclass
class TestResult:
    joint: str
    direction: int
    command_velocity: float
    measured_velocity: float
    position_change: float
    estimated_pwm: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MinimumMotionTester(Node):

    def __init__(self) -> None:
        super().__init__("minimum_motion_tester")

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            COMMAND_TOPIC,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.lock = threading.Lock()
        self.positions: Dict[str, float] = {}
        self.velocities: Dict[str, float] = {}
        self.last_joint_state_time: Optional[float] = None

        self.results: List[TestResult] = []

        self.get_logger().info("Minimum-motion tester ready")
        self.get_logger().warn(
            "Tests one joint at a time. Keep the emergency stop accessible."
        )

    def joint_state_callback(self, msg: JointState) -> None:
        now = time.monotonic()

        with self.lock:
            for index, name in enumerate(msg.name):
                if name not in JOINT_NAMES:
                    continue

                if index < len(msg.position):
                    self.positions[name] = float(msg.position[index])

                if index < len(msg.velocity):
                    self.velocities[name] = float(msg.velocity[index])

            self.last_joint_state_time = now

    def get_state(
        self,
    ) -> Tuple[Dict[str, float], Dict[str, float], Optional[float]]:
        with self.lock:
            return (
                dict(self.positions),
                dict(self.velocities),
                self.last_joint_state_time,
            )

    def wait_for_joint_states(self) -> bool:
        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            positions, _, last_time = self.get_state()

            complete = all(
                joint in positions
                for joint in JOINT_NAMES
            )

            fresh = (
                last_time is not None
                and time.monotonic() - last_time
                <= JOINT_STATE_STALE_TIMEOUT_S
            )

            if complete and fresh:
                return True

            if time.monotonic() - start > WAIT_FOR_JOINT_STATES_S:
                return False

        return False

    def publish_command(self, values: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        self.command_pub.publish(msg)

    def stop(self, duration_s: float = 0.25) -> None:
        end = time.monotonic() + duration_s

        while rclpy.ok() and time.monotonic() < end:
            self.publish_command([0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(COMMAND_PERIOD)

    def command_for_joint(
        self,
        joint: str,
        velocity: float,
    ) -> List[float]:
        command = [0.0, 0.0, 0.0]
        command[JOINT_NAMES.index(joint)] = velocity
        return command

    def estimate_pwm(
        self,
        joint: str,
        signed_command_velocity: float,
        measured_velocity: float,
    ) -> float:
        sign = 1.0 if signed_command_velocity > 0.0 else -1.0

        desired_speed = clamp(
            abs(signed_command_velocity),
            MIN_COMMAND_VELOCITY[joint],
            MAX_JOINT_VELOCITY[joint],
        )

        measured_speed_along_direction = measured_velocity * sign
        velocity_error = desired_speed - measured_speed_along_direction

        pwm_abs = (
            MIN_PWM[joint]
            + VELOCITY_TO_PWM_GAIN[joint] * desired_speed
            + VELOCITY_KP[joint] * velocity_error
        )

        return clamp(
            pwm_abs,
            0.0,
            MAX_PWM[joint],
        )

    def safe_for_direction(
        self,
        joint: str,
        direction: int,
        position: float,
    ) -> bool:
        lower, upper = JOINT_LIMITS[joint]
        margin = LIMIT_MARGIN_RAD[joint]

        if direction < 0:
            return position > lower + margin

        return position < upper - margin

    def test_step(
        self,
        joint: str,
        direction: int,
        speed: float,
    ) -> Optional[TestResult]:
        positions, _, _ = self.get_state()
        start_position = positions[joint]

        if not self.safe_for_direction(
            joint,
            direction,
            start_position,
        ):
            self.get_logger().warn(
                f"{joint}: direction {direction:+d} skipped near joint limit"
            )
            return None

        signed_speed = float(direction) * speed
        command = self.command_for_joint(
            joint,
            signed_speed,
        )

        start_time = time.monotonic()
        moving_samples = 0
        max_position_change = 0.0
        measured_velocity_samples: List[float] = []

        while rclpy.ok():
            now = time.monotonic()

            if now - start_time >= STEP_DURATION_S:
                break

            self.publish_command(command)
            rclpy.spin_once(self, timeout_sec=0.002)

            positions, velocities, last_time = self.get_state()

            if (
                last_time is None
                or now - last_time > JOINT_STATE_STALE_TIMEOUT_S
            ):
                self.get_logger().error("Joint states became stale")
                self.stop()
                return None

            current_position = positions[joint]
            position_change = abs(current_position - start_position)
            max_position_change = max(
                max_position_change,
                position_change,
            )

            measured_velocity = velocities.get(joint, 0.0)

            if position_change >= MIN_POSITION_CHANGE_RAD[joint]:
                moving_samples += 1
                measured_velocity_samples.append(measured_velocity)

                if moving_samples >= MIN_MOVING_SAMPLES:
                    confirm_end = time.monotonic() + CONFIRM_MOTION_TIME_S

                    while rclpy.ok() and time.monotonic() < confirm_end:
                        self.publish_command(command)
                        rclpy.spin_once(self, timeout_sec=0.002)

                        _, confirm_velocities, _ = self.get_state()
                        measured_velocity_samples.append(
                            confirm_velocities.get(joint, 0.0)
                        )

                        time.sleep(COMMAND_PERIOD)

                    self.stop()

                    nonzero_samples = [
                        value
                        for value in measured_velocity_samples
                        if abs(value) > 1e-6
                    ]

                    if nonzero_samples:
                        average_measured_velocity = (
                            sum(nonzero_samples)
                            / len(nonzero_samples)
                        )
                    else:
                        average_measured_velocity = 0.0

                    estimated_pwm = self.estimate_pwm(
                        joint,
                        signed_speed,
                        average_measured_velocity,
                    )

                    return TestResult(
                        joint=joint,
                        direction=direction,
                        command_velocity=signed_speed,
                        measured_velocity=average_measured_velocity,
                        position_change=max_position_change,
                        estimated_pwm=estimated_pwm,
                    )

            if not self.safe_for_direction(
                joint,
                direction,
                current_position,
            ):
                self.get_logger().warn(
                    f"{joint}: stopping because joint-limit margin was reached"
                )
                self.stop()
                return None

            time.sleep(COMMAND_PERIOD)

        self.stop()
        return None

    def test_direction(
        self,
        joint: str,
        direction: int,
    ) -> Optional[TestResult]:
        direction_name = "positive" if direction > 0 else "negative"

        self.get_logger().warn(
            f"Testing {joint} in {direction_name} direction"
        )

        for speed in TEST_VELOCITIES[joint]:
            positions, _, _ = self.get_state()

            if not self.safe_for_direction(
                joint,
                direction,
                positions[joint],
            ):
                self.get_logger().warn(
                    f"{joint}: no room left for {direction_name} test"
                )
                return None

            estimated_no_motion_pwm = self.estimate_pwm(
                joint,
                float(direction) * speed,
                0.0,
            )

            print(
                f"  test {joint:14s} "
                f"dir={direction:+d} "
                f"cmd={direction * speed:+.4f} rad/s "
                f"estimated_pwm={estimated_no_motion_pwm:.4f}"
            )

            result = self.test_step(
                joint,
                direction,
                speed,
            )

            if result is not None:
                self.results.append(result)

                self.get_logger().warn(
                    f"MOTION DETECTED: {joint} dir={direction:+d}, "
                    f"minimum tested command={result.command_velocity:+.4f} rad/s, "
                    f"estimated PWM={result.estimated_pwm:.4f}"
                )

                time.sleep(SETTLE_TIME_S)
                return result

            time.sleep(SETTLE_TIME_S)

        self.get_logger().error(
            f"No movement detected for {joint} direction {direction:+d}"
        )
        return None

    def run_all_tests(self) -> None:
        if not self.wait_for_joint_states():
            self.get_logger().error(
                "No complete and fresh /joint_states received"
            )
            return

        print("")
        print("Current positions:")

        positions, _, _ = self.get_state()

        for joint in JOINT_NAMES:
            print(f"  {joint}: {positions[joint]:+.4f} rad")

        print("")
        print("The test will move one joint at a time.")
        print("Enter y only when the robot is clear and E-stop is accessible.")
        print("")

        confirmation = input("Start automatic test? [y/N]: ").strip().lower()

        if confirmation != "y":
            self.stop()
            return

        try:
            for joint in JOINT_NAMES:
                # Test the direction with more available travel first.
                positions, _, _ = self.get_state()
                current = positions[joint]
                lower, upper = JOINT_LIMITS[joint]

                positive_room = upper - current
                negative_room = current - lower

                directions = (
                    [1, -1]
                    if positive_room >= negative_room
                    else [-1, 1]
                )

                for direction in directions:
                    self.stop()
                    time.sleep(SETTLE_TIME_S)

                    self.test_direction(
                        joint,
                        direction,
                    )

        finally:
            self.stop()

        self.print_summary()

    def print_summary(self) -> None:
        print("")
        print("=" * 78)
        print("MINIMUM MOTION TEST RESULTS")
        print("=" * 78)

        for joint in JOINT_NAMES:
            for direction in (-1, 1):
                matching = [
                    result
                    for result in self.results
                    if result.joint == joint
                    and result.direction == direction
                ]

                direction_name = (
                    "negative"
                    if direction < 0
                    else "positive"
                )

                if not matching:
                    print(
                        f"{joint:14s} {direction_name:8s}: "
                        "no movement found"
                    )
                    continue

                result = matching[0]

                duty_percent = (
                    result.estimated_pwm
                    / MAX_PWM[joint]
                    * 100.0
                )

                print(
                    f"{joint:14s} {direction_name:8s}: "
                    f"cmd={result.command_velocity:+.4f} rad/s, "
                    f"measured={result.measured_velocity:+.4f} rad/s, "
                    f"estimated_pwm={result.estimated_pwm:.4f}, "
                    f"duty={duty_percent:.1f}%"
                )

        print("=" * 78)
        print("")
        print(
            "Use the highest absolute threshold from both directions "
            "as a conservative starting value."
        )
        print(
            "The PWM result is an estimate from the current hardware formula, "
            "not a direct raw-PWM measurement."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimumMotionTester()

    try:
        node.run_all_tests()

    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user")

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
