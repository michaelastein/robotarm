#!/usr/bin/env python3

import threading
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


# -----------------------------
# Frames
# -----------------------------

BASE_FRAME = "base_link"
TIP_FRAME = "tool_tip_link"


# -----------------------------
# Detection parameters for LED testing at home
# -----------------------------

ROI_SIZE = 120
LOST_FRAMES_RESET = 8
MAX_TRAIL = 120

CONF_THRESHOLD = 2.0

MIN_BRIGHTNESS = 150
MIN_SATURATION = 40

MAX_HOTSPOT_AREA_FRACTION = 0.03
MIN_HOTSPOT_PIXELS = 3


# -----------------------------
# Servo parameters: slow debug mode
# -----------------------------

CONTROL_RATE = 50.0

# Larger deadband = less jitter near image center.
DEADBAND = 0.12

# Slow gain.
KP_X = 0.010

# Image-Y is intentionally ignored for now.
KP_Y = 0.0

# Increased because final command is multiplied.
MAX_VEL = 0.015

# Smooth command.
SMOOTHING_ALPHA = 0.20
LOST_ALPHA = 0.35

LOST_TIMEOUT = 10

# Do not publish extremely tiny motion commands.
COMMAND_EPSILON = 0.00002

# Multiply final command before publishing.
# Start with 3.0. Try 5.0 only if still too weak.
COMMAND_MULTIPLIER = 3.0

# After target is lost, send this many zero commands, then stop publishing.
STOP_COMMANDS_AFTER_LOST = 5

# Print debug once per second.
DEBUG_PRINT_PERIOD = 1.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def find_hotspot(img):
    """
    LED-friendly hotspot detection.

    Uses brightness, saturation, percentile threshold,
    area filtering, and weighted centroid.
    """
    if img is None:
        return None, 0.0, None

    h, w = img.shape[:2]
    total = h * w

    blurred = cv2.GaussianBlur(img, (5, 5), 0)

    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    saturation = hsv[:, :, 1].astype(np.float32)

    mean_gray = float(np.mean(gray))
    max_gray = float(np.max(gray))

    if max_gray < MIN_BRIGHTNESS:
        return None, 0.0, None

    p990 = float(np.percentile(gray, 99.0))

    bright_thr = max(
        MIN_BRIGHTNESS,
        p990,
        mean_gray + 35.0,
    )

    bright_mask = gray >= bright_thr
    sat_mask = saturation >= MIN_SATURATION

    mask = bright_mask | ((gray >= mean_gray + 25.0) & sat_mask)

    mask_uint8 = mask.astype(np.uint8) * 255
    mask_uint8 = cv2.morphologyEx(
        mask_uint8,
        cv2.MORPH_OPEN,
        np.ones((3, 3), np.uint8),
    )

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask_uint8,
        connectivity=8,
    )

    best_label = None
    best_score = 0.0

    for label in range(1, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]

        if area < MIN_HOTSPOT_PIXELS:
            continue

        if area > MAX_HOTSPOT_AREA_FRACTION * total:
            continue

        bw = stats[label, cv2.CC_STAT_WIDTH]
        bh = stats[label, cv2.CC_STAT_HEIGHT]

        component_mask = labels == label

        mean_hot = float(np.mean(gray[component_mask]))
        max_hot = float(np.max(gray[component_mask]))
        mean_sat = float(np.mean(saturation[component_mask]))

        contrast = mean_hot / (mean_gray + 1e-6)

        compactness_bonus = 1.0 / max(1.0, np.sqrt(area))
        brightness_score = max_hot / 255.0
        contrast_score = max(0.5, contrast)
        saturation_score = 1.0 + (mean_sat / 255.0)

        score = (
            brightness_score
            * contrast_score
            * saturation_score
            * min(3.0, area / 8.0)
            * (1.0 + compactness_bonus)
        )

        # Penalize very large reflections.
        if bw > 0.25 * w or bh > 0.25 * h:
            score *= 0.2

        if score > best_score:
            best_score = score
            best_label = label

    if best_label is None:
        return None, 0.0, mask_uint8

    component_mask = labels == best_label
    ys, xs = np.where(component_mask)

    if len(xs) == 0:
        return None, 0.0, mask_uint8

    weights = gray[ys, xs]
    weights = weights / (np.sum(weights) + 1e-6)

    cx = float(np.sum(xs * weights))
    cy = float(np.sum(ys * weights))

    meas = np.array([[cx], [cy]], dtype=np.float32)

    return meas, float(best_score), mask_uint8


class HotspotVisualServo(Node):

    def __init__(self):
        super().__init__("hotspot_visual_servo")

        self.sub = self.create_subscription(
            CompressedImage,
            "/cam0/camera/image_raw/compressed",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.annotated_pub = self.create_publisher(
            CompressedImage,
            "/hotspot_visual_servo/annotated_image/compressed",
            qos_profile_sensor_data,
        )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        self.servo_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.lock = threading.Lock()
        self.latest_msg = None

        self.kalman = cv2.KalmanFilter(4, 2)

        self.kalman.measurementMatrix = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            dtype=np.float32,
        )

        self.kalman.transitionMatrix = np.array(
            [
                [1, 0, 1, 0],
                [0, 1, 0, 1],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.02
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0

        self.kalman.statePost = np.array(
            [[0.0], [0.0], [0.0], [0.0]],
            dtype=np.float32,
        )

        self.traj = []
        self.last_valid = None
        self.lost_counter = 999

        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0

        self.current_error_x = 0.0
        self.current_error_y = 0.0
        self.current_conf = 0.0
        self.target_visible = False
        self.mode = "FULL"

        self.last_debug_time = 0.0
        self.last_tip_pos = None

        self.lost_stop_publish_count = STOP_COMMANDS_AFTER_LOST

        self.switch_to_twist_mode()

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE,
            self.publish_servo_command,
        )

        self.get_logger().info("LED hotspot visual servo DEBUG started")
        self.get_logger().info(
            "Annotated image topic: "
            "/hotspot_visual_servo/annotated_image/compressed"
        )
        self.get_logger().info(
            f"Expected MoveIt chain tip from SRDF: {TIP_FRAME}"
        )
        self.get_logger().info(
            f"Twist command frame_id: {BASE_FRAME}"
        )
        self.get_logger().warn(
            "DEBUG MODE: only image-x controls base_link linear.y. "
            "linear.x=0 and linear.z=0."
        )
        self.get_logger().warn(
            f"Command multiplier active: {COMMAND_MULTIPLIER:.1f}"
        )

    def switch_to_twist_mode(self):
        while rclpy.ok() and not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Servo command-type service...")

        if not rclpy.ok():
            return

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST

        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("MoveIt Servo switched to TWIST mode")
        else:
            self.get_logger().warn("Could not switch MoveIt Servo to TWIST mode")

    def image_callback(self, msg):
        with self.lock:
            self.latest_msg = msg

    def get_tip_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                TIP_FRAME,
                rclpy.time.Time(),
            )

            t = transform.transform.translation

            return (
                float(t.x),
                float(t.y),
                float(t.z),
            )

        except TransformException:
            return None

    def create_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE_FRAME

        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        return twist

    def publish_zero_once(self):
        self.twist_pub.publish(self.create_zero_twist())

    def maybe_debug_print(self, did_publish=False, reason=""):
        now = time.monotonic()

        if now - self.last_debug_time < DEBUG_PRINT_PERIOD:
            return

        self.last_debug_time = now

        tip_pos = self.get_tip_position()

        if tip_pos is None:
            tip_text = "tip=unknown"
            dz_text = "d=?"
        else:
            x, y, z = tip_pos

            if self.last_tip_pos is None:
                dx = 0.0
                dy = 0.0
                dz = 0.0
            else:
                dx = x - self.last_tip_pos[0]
                dy = y - self.last_tip_pos[1]
                dz = z - self.last_tip_pos[2]

            self.last_tip_pos = tip_pos

            tip_text = (
                f"tip x={x:+.4f} y={y:+.4f} z={z:+.4f}"
            )
            dz_text = (
                f"d={dx:+.4f},{dy:+.4f},{dz:+.4f}"
            )

        self.get_logger().info(
            f"{tip_text} | {dz_text} | "
            f"visible={self.target_visible} "
            f"mode={self.mode} "
            f"conf={self.current_conf:.2f} "
            f"err=({self.current_error_x:+.3f},{self.current_error_y:+.3f}) | "
            f"cmd=({self.filtered_vx:+.5f},"
            f"{self.filtered_vy:+.5f},"
            f"{self.filtered_vz:+.5f}) | "
            f"published={did_publish} reason={reason} "
            f"lost_stop_count={self.lost_stop_publish_count}"
        )

    def update_from_image(self, img, input_msg):
        h, w = img.shape[:2]

        roi_rect = None

        if self.last_valid is not None and self.lost_counter < LOST_FRAMES_RESET:
            x, y = self.last_valid

            x1 = max(0, x - ROI_SIZE)
            y1 = max(0, y - ROI_SIZE)
            x2 = min(w, x + ROI_SIZE)
            y2 = min(h, y + ROI_SIZE)

            search_img = img[y1:y2, x1:x2]
            offset = np.array([x1, y1], dtype=np.float32)
            self.mode = "ROI"
            roi_rect = (x1, y1, x2, y2)
        else:
            search_img = img
            offset = np.array([0.0, 0.0], dtype=np.float32)
            self.mode = "FULL"

        self.kalman.predict()

        meas, conf, _ = find_hotspot(search_img)
        self.current_conf = float(conf)

        valid = meas is not None and conf >= CONF_THRESHOLD

        if valid:
            meas = np.array(meas, dtype=np.float32, copy=True)

            meas[0, 0] += offset[0]
            meas[1, 0] += offset[1]

            noise = np.float32(max(0.05, 2.0 / (conf + 1e-3)))
            self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * noise

            self.kalman.correct(meas)

            self.last_valid = (
                int(meas[0, 0]),
                int(meas[1, 0]),
            )

            self.lost_counter = 0
            self.target_visible = True
        else:
            self.lost_counter += 1

            if self.lost_counter > LOST_TIMEOUT:
                self.target_visible = False
                self.current_error_x = 0.0
                self.current_error_y = 0.0

        est = self.kalman.statePost.ravel().astype(np.float32)

        if self.target_visible:
            px = float(est[0])
            py = float(est[1])

            self.current_error_x = (px - (w * 0.5)) / (w * 0.5)
            self.current_error_y = (py - (h * 0.5)) / (h * 0.5)

            pt = (int(px), int(py))

            self.traj.append(pt)

            if len(self.traj) > MAX_TRAIL:
                self.traj.pop(0)

        #annotated = self.make_annotated_image(
         #   img,
          #  roi_rect=roi_rect,
           # valid=valid,
        #)

        #self.publish_annotated_image(annotated, input_msg)

    def make_annotated_image(self, img, roi_rect=None, valid=False):
        h, w = img.shape[:2]
        vis = img.copy()

        center = (w // 2, h // 2)

        for i in range(1, len(self.traj)):
            cv2.line(
                vis,
                self.traj[i - 1],
                self.traj[i],
                (255, 0, 0),
                2,
            )

        if roi_rect is not None:
            x1, y1, x2, y2 = roi_rect

            cv2.rectangle(
                vis,
                (x1, y1),
                (x2, y2),
                (0, 255, 255),
                2,
            )

        if self.target_visible and self.last_valid is not None:
            pt = self.last_valid
        else:
            pt = center

        if valid:
            color = (0, 0, 255)
            status = "LED HOTSPOT"
        elif self.target_visible:
            color = (0, 165, 255)
            status = "TRACKING PREDICTED"
        else:
            color = (100, 100, 100)
            status = "NO LED"

        cv2.circle(vis, pt, 8, color, -1)
        cv2.circle(vis, center, 8, (0, 255, 0), 2)

        if self.last_valid is not None:
            cv2.line(vis, center, self.last_valid, (255, 255, 0), 2)

        status_text = (
            f"{status} mode={self.mode} "
            f"conf={self.current_conf:.2f} "
            f"lost={self.lost_counter}"
        )

        error_text = (
            f"err=({self.current_error_x:+.2f}, {self.current_error_y:+.2f}) "
            f"visible={self.target_visible}"
        )

        velocity_text = (
            f"cmd=({self.filtered_vx:+.3f},"
            f"{self.filtered_vy:+.3f},"
            f"{self.filtered_vz:+.3f}) "
            f"mult={COMMAND_MULTIPLIER:.1f} only image-x -> base Y"
        )

        cv2.putText(
            vis,
            status_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            vis,
            error_text,
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            vis,
            velocity_text,
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        return vis

    def publish_annotated_image(self, img, input_msg):
        ok, encoded = cv2.imencode(
            ".jpg",
            img,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80],
        )

        if not ok:
            self.get_logger().warn("Could not encode annotated image")
            return

        out = CompressedImage()
        out.header = input_msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()

        self.annotated_pub.publish(out)

    def publish_servo_command(self):
        with self.lock:
            msg = self.latest_msg
            self.latest_msg = None

        if msg is not None:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                self.update_from_image(img, msg)

        raw_vx = 0.0
        raw_vy = 0.0
        raw_vz = 0.0

        alpha = SMOOTHING_ALPHA

        if not self.target_visible:
            alpha = LOST_ALPHA

        if self.target_visible:
            err_x = self.current_error_x

            if abs(err_x) < DEADBAND:
                err_x = 0.0

            # DEBUG TEST MODE:
            # Only image x error is allowed to move the robot.
            # Image y is ignored completely.
            raw_vx = 0.0
            raw_vy = KP_X * err_x
            raw_vz = 0.0

            raw_vy = clamp(raw_vy, -MAX_VEL, MAX_VEL)

        self.filtered_vx = (
            alpha * raw_vx
            + (1.0 - alpha) * self.filtered_vx
        )

        self.filtered_vy = (
            alpha * raw_vy
            + (1.0 - alpha) * self.filtered_vy
        )

        # Hard height lock: never publish vertical velocity.
        self.filtered_vz = 0.0

        # Multiply final command before publishing.
        self.filtered_vx *= COMMAND_MULTIPLIER
        self.filtered_vy *= COMMAND_MULTIPLIER

        # Clamp again after multiplying.
        self.filtered_vx = clamp(self.filtered_vx, -MAX_VEL, MAX_VEL)
        self.filtered_vy = clamp(self.filtered_vy, -MAX_VEL, MAX_VEL)
        self.filtered_vz = 0.0

        # Remove extremely tiny residual commands.
        if abs(self.filtered_vx) < COMMAND_EPSILON:
            self.filtered_vx = 0.0

        if abs(self.filtered_vy) < COMMAND_EPSILON:
            self.filtered_vy = 0.0

        if abs(self.filtered_vz) < COMMAND_EPSILON:
            self.filtered_vz = 0.0

        # If no target is visible:
        # publish a few stop commands, then publish nothing.
        if not self.target_visible:
            self.filtered_vx = 0.0
            self.filtered_vy = 0.0
            self.filtered_vz = 0.0

            if self.lost_stop_publish_count < STOP_COMMANDS_AFTER_LOST:
                self.publish_zero_once()
                self.lost_stop_publish_count += 1
                self.maybe_debug_print(
                    did_publish=True,
                    reason="lost_target_zero_stop",
                )
            else:
                self.maybe_debug_print(
                    did_publish=False,
                    reason="lost_target_no_publish",
                )

            return

        # If target is visible but command is zero/tiny:
        # publish nothing. Do not spam Servo with zero Cartesian commands.
        if (
            abs(self.filtered_vx) < COMMAND_EPSILON
            and abs(self.filtered_vy) < COMMAND_EPSILON
            and abs(self.filtered_vz) < COMMAND_EPSILON
        ):
            self.maybe_debug_print(
                did_publish=False,
                reason="visible_but_tiny_no_publish",
            )
            return

        self.lost_stop_publish_count = 0

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE_FRAME

        twist.twist.linear.x = self.filtered_vx
        twist.twist.linear.y = self.filtered_vy
        twist.twist.linear.z = 0.0

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        self.twist_pub.publish(twist)

        self.get_logger().warn(
            f"PUBLISH TWIST: "
            f"visible={self.target_visible} "
            f"err=({self.current_error_x:+.3f},{self.current_error_y:+.3f}) "
            f"cmd=({self.filtered_vx:+.5f},"
            f"{self.filtered_vy:+.5f},"
            f"{self.filtered_vz:+.5f})",
            throttle_duration_sec=0.5,
        )

        self.maybe_debug_print(
            did_publish=True,
            reason="active_tracking",
        )


def main(args=None):
    rclpy.init(args=args)

    node = HotspotVisualServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_zero_once()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
