#!/usr/bin/env python3

import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray


# -----------------------------
# Detection parameters
# -----------------------------

# Complete ROI side length as a fraction of image width.
# 0.20 means the ROI is one fifth of the full image width.
ROI_WIDTH_FRACTION = 0.35
LOST_FRAMES_RESET = 8
MAX_TRAIL = 120

CONF_THRESHOLD = 2.0

MIN_BRIGHTNESS = 150
MIN_SATURATION = 40

MAX_HOTSPOT_AREA_FRACTION = 0.03
MIN_HOTSPOT_PIXELS = 3

# Publish annotated image only every Nth frame.
ANNOTATION_EVERY_N_FRAMES = 5



def fit_text_scale(
    text,
    max_width,
    font=cv2.FONT_HERSHEY_SIMPLEX,
    preferred_scale=0.55,
    min_scale=0.28,
    thickness=1,
):
    """Return the largest font scale that keeps text inside max_width."""
    scale = preferred_scale

    while scale > min_scale:
        (text_width, _), _ = cv2.getTextSize(
            text,
            font,
            scale,
            thickness,
        )

        if text_width <= max_width:
            break

        scale -= 0.02

    return max(scale, min_scale)

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


class HotspotDetector(Node):

    def __init__(self):
        super().__init__("hotspot_detector")

        self.sub = self.create_subscription(
            CompressedImage,
            "/cam0/camera/image_raw/compressed",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.target_pub = self.create_publisher(
            Float32MultiArray,
            "/hotspot/target",
            10,
        )

        self.annotated_pub = self.create_publisher(
            CompressedImage,
            "/hotspot/annotated_image/compressed",
            qos_profile_sensor_data,
        )

        self.lock = threading.Lock()

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

        self.current_error_x = 0.0
        self.current_error_y = 0.0
        self.current_conf = 0.0
        self.target_visible = False
        self.current_detection_valid = False
        self.mode = "FULL"

        self.frame_count = 0

        self.get_logger().info("Hotspot detector started")
        self.get_logger().info("Target topic: /hotspot/target")
        self.get_logger().info("Annotated image topic: /hotspot/annotated_image/compressed")
        self.get_logger().info(
            f"Annotated image published every {ANNOTATION_EVERY_N_FRAMES} frames"
        )

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            return

        self.frame_count += 1

        self.process_image(img, msg)

    def publish_target(self):
        msg = Float32MultiArray()

        visible_value = 1.0 if self.target_visible and self.current_detection_valid else 0.0

        msg.data = [
            float(visible_value),
            float(self.current_error_x),
            float(self.current_error_y),
            float(self.current_conf),
        ]

        self.target_pub.publish(msg)

    def process_image(self, img, input_msg):
        h, w = img.shape[:2]

        roi_rect = None

        if self.last_valid is not None and self.lost_counter < LOST_FRAMES_RESET:
            x, y = self.last_valid

            # Adaptive square ROI: complete side length is 1/5 of image width.
            roi_side = max(2, int(round(w * ROI_WIDTH_FRACTION)))
            roi_half = max(1, roi_side // 2)

            x1 = max(0, x - roi_half)
            y1 = max(0, y - roi_half)
            x2 = min(w, x + roi_half)
            y2 = min(h, y + roi_half)

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
        self.current_detection_valid = valid

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

            if self.lost_counter > LOST_FRAMES_RESET:
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

        self.publish_target()

        if self.frame_count % ANNOTATION_EVERY_N_FRAMES == 0:
            annotated = self.make_annotated_image(
                img,
                roi_rect=roi_rect,
                valid=valid,
            )

            self.publish_annotated_image(annotated, input_msg)

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
            f"valid={self.current_detection_valid}"
        )

        # Scale each line to the current image width so compressed or
        # low-resolution streams do not clip the annotation text.
        text_margin = 10
        max_text_width = max(1, w - (2 * text_margin))
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_thickness = 1

        status_scale = fit_text_scale(
            status_text,
            max_text_width,
            font=font,
            preferred_scale=0.55,
            min_scale=0.28,
            thickness=text_thickness,
        )
        error_scale = fit_text_scale(
            error_text,
            max_text_width,
            font=font,
            preferred_scale=0.55,
            min_scale=0.28,
            thickness=text_thickness,
        )

        (_, status_height), status_baseline = cv2.getTextSize(
            status_text, font, status_scale, text_thickness
        )
        (_, error_height), error_baseline = cv2.getTextSize(
            error_text, font, error_scale, text_thickness
        )

        status_y = text_margin + status_height
        error_y = status_y + status_baseline + error_height + 8

        # A subtle dark panel keeps the smaller text readable.
        panel_bottom = min(h, error_y + error_baseline + 6)
        overlay = vis.copy()
        cv2.rectangle(
            overlay,
            (0, 0),
            (w, panel_bottom),
            (0, 0, 0),
            -1,
        )
        cv2.addWeighted(overlay, 0.40, vis, 0.60, 0, vis)

        cv2.putText(
            vis,
            status_text,
            (text_margin, status_y),
            font,
            status_scale,
            (255, 255, 255),
            text_thickness,
            cv2.LINE_AA,
        )

        cv2.putText(
            vis,
            error_text,
            (text_margin, error_y),
            font,
            error_scale,
            (255, 255, 255),
            text_thickness,
            cv2.LINE_AA,
        )

        return vis

    def publish_annotated_image(self, img, input_msg):
        ok, encoded = cv2.imencode(
            ".jpg",
            img,
            [int(cv2.IMWRITE_JPEG_QUALITY), 75],
        )

        if not ok:
            self.get_logger().warn("Could not encode annotated image")
            return

        out = CompressedImage()
        out.header = input_msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()

        self.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = HotspotDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
