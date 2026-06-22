#!/usr/bin/env python3

import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


CONF_THRESHOLD = 5.0

CONTROL_RATE = 50.0
DEADBAND = 0.07

KP_X = 0.02
KP_Y = 0.02

MAX_VEL = 0.025
SMOOTHING_ALPHA = 0.10
LOST_ALPHA = 0.35

LOST_TIMEOUT = 10


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def find_hotspot(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    mean_all = np.mean(gray)
    max_val = np.max(gray)

    if max_val < 180:
        return None, 0.0

    global_thr = max(235, mean_all + 40)
    mask = gray >= global_thr

    count = np.sum(mask)

    if count < 3 or count > 0.01 * gray.size:
        return None, 0.0

    ys, xs = np.where(mask)

    if len(xs) == 0:
        return None, 0.0

    weights = gray[ys, xs]
    weights = weights / (np.sum(weights) + 1e-6)

    cx = np.sum(xs * weights)
    cy = np.sum(ys * weights)

    hot_mean = np.mean(gray[ys, xs])
    contrast = hot_mean / (mean_all + 1e-6)

    confidence = float(
        (hot_mean / 255.0)
        * max(1.0, contrast)
        * (count / 10.0)
    )

    return np.array([cx, cy], dtype=np.float32), confidence


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

        self.lock = threading.Lock()
        self.latest_msg = None

        self.last_valid = None
        self.lost_counter = 0

        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0

        self.current_error_x = 0.0
        self.current_error_y = 0.0
        self.current_conf = 0.0
        self.target_visible = False

        self.switch_to_twist_mode()

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE,
            self.publish_servo_command,
        )

        self.get_logger().info("Hotspot visual servo started")
        self.get_logger().info(
            "Annotated image topic: "
            "/hotspot_visual_servo/annotated_image/compressed"
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

    def update_from_image(self, img, input_msg):
        h, w = img.shape[:2]
        meas, conf = find_hotspot(img)

        if meas is not None and conf >= CONF_THRESHOLD:
            x = meas[0]
            y = meas[1]

            nx = (x - (w * 0.5)) / (w * 0.5)
            ny = (y - (h * 0.5)) / (h * 0.5)

            self.current_error_x = float(nx)
            self.current_error_y = float(ny)
            self.current_conf = float(conf)

            self.last_valid = (int(x), int(y))
            self.lost_counter = 0
            self.target_visible = True
        else:
            self.lost_counter += 1
            self.current_conf = float(conf)

            if self.lost_counter > LOST_TIMEOUT:
                self.target_visible = False
                self.current_error_x = 0.0
                self.current_error_y = 0.0

        annotated = self.make_annotated_image(img)
        self.publish_annotated_image(annotated, input_msg)

    def make_annotated_image(self, img):
        h, w = img.shape[:2]
        vis = img.copy()

        center = (w // 2, h // 2)

        if self.last_valid is not None and self.target_visible:
            pt = self.last_valid
            target_color = (0, 0, 255)
        elif self.last_valid is not None:
            pt = self.last_valid
            target_color = (0, 165, 255)
        else:
            pt = center
            target_color = (0, 165, 255)

        cv2.circle(vis, pt, 8, target_color, -1)
        cv2.circle(vis, center, 8, (0, 255, 0), 2)

        if self.last_valid is not None:
            cv2.line(vis, center, pt, (255, 255, 0), 2)

        status = (
            f"visible={self.target_visible} "
            f"err=({self.current_error_x:+.2f}, {self.current_error_y:+.2f}) "
            f"conf={self.current_conf:.2f} "
            f"lost={self.lost_counter}"
        )

        velocity_status = (
            f"vel=({self.filtered_vx:+.3f}, "
            f"{self.filtered_vy:+.3f}, "
            f"{self.filtered_vz:+.3f})"
        )

        cv2.putText(
            vis,
            status,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            vis,
            velocity_status,
            (10, 50),
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
            err_y = self.current_error_y

            if abs(err_x) < DEADBAND:
                err_x = 0.0

            if abs(err_y) < DEADBAND:
                err_y = 0.0

            # Camera mounted on end-effector.
            # Image x error controls sideways motion.
            # Image y error controls vertical motion.
            #
            # Depending on camera orientation, you may need to flip signs.
            raw_vy = -KP_X * err_x
            raw_vz = -KP_Y * err_y

            raw_vy = clamp(raw_vy, -MAX_VEL, MAX_VEL)
            raw_vz = clamp(raw_vz, -MAX_VEL, MAX_VEL)

        self.filtered_vx = (
            alpha * raw_vx
            + (1.0 - alpha) * self.filtered_vx
        )

        self.filtered_vy = (
            alpha * raw_vy
            + (1.0 - alpha) * self.filtered_vy
        )

        self.filtered_vz = (
            alpha * raw_vz
            + (1.0 - alpha) * self.filtered_vz
        )

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        twist.twist.linear.x = self.filtered_vx
        twist.twist.linear.y = self.filtered_vy
        twist.twist.linear.z = self.filtered_vz

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = HotspotVisualServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
