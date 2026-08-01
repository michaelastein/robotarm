#!/usr/bin/env python3
"""ROS 2 node for detecting and tracking a bright LED hotspot.

Configuration parameters:
    ROI_WIDTH_FRACTION:
        Side length of the square tracking region of interest (ROI), expressed
        as a fraction of the full image width. For example, 0.35 creates an ROI
        whose width and height are each 35% of the source image width. The ROI
        is centered on the most recent valid detection and reduces the search
        area while the target is being tracked.

    LOST_FRAMES_RESET:
        Number of consecutive frames without a valid detection that may pass
        before ROI tracking is abandoned. After this limit is exceeded, the
        detector returns to a full-frame search and reports the target as not
        visible.

    MAX_TRAIL:
        Maximum number of estimated target positions retained for the trail
        drawn on annotated images. Older points are discarded first.

    CONF_THRESHOLD:
        Minimum hotspot score required for a measurement to be accepted as a
        valid detection. Higher values make detection stricter; lower values
        make it more permissive.

    MIN_BRIGHTNESS:
        Minimum grayscale intensity, in the OpenCV range 0-255, that must occur
        in the search image before hotspot analysis continues.

    MIN_SATURATION:
        Minimum HSV saturation, in the OpenCV range 0-255, used to retain
        bright, colored LED candidates that may not meet the strictest
        brightness threshold.

    MAX_HOTSPOT_AREA_FRACTION:
        Largest connected-component area accepted as a hotspot, expressed as a
        fraction of the current search image area. This rejects large bright
        regions such as reflections or overexposed backgrounds.

    MIN_HOTSPOT_PIXELS:
        Smallest connected-component area, in pixels, accepted as a hotspot.
        This suppresses isolated sensor noise and tiny compression artifacts.

    ANNOTATION_EVERY_N_FRAMES:
        Publish one annotated JPEG for every N input frames. Target coordinates
        are still published for every successfully decoded input frame.

Published target message layout:
    Float32MultiArray.data = [visible, error_x, error_y, confidence]

    visible:
        1.0 when the current frame contains a valid detection and tracking is
        active; otherwise 0.0.

    error_x, error_y:
        Estimated target displacement from the image center, normalized to
        approximately [-1.0, 1.0]. Positive x is right; positive y is down.

    confidence:
        Raw score assigned to the strongest hotspot candidate.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray


# Tracking and detection configuration.
ROI_WIDTH_FRACTION = 0.35
LOST_FRAMES_RESET = 8
MAX_TRAIL = 120

CONF_THRESHOLD = 2.0
MIN_BRIGHTNESS = 150
MIN_SATURATION = 40
MAX_HOTSPOT_AREA_FRACTION = 0.03
MIN_HOTSPOT_PIXELS = 3

ANNOTATION_EVERY_N_FRAMES = 5


def fit_text_scale(
    text,
    max_width,
    font=cv2.FONT_HERSHEY_SIMPLEX,
    preferred_scale=0.55,
    min_scale=0.28,
    thickness=1,
):
    """Find a font scale that keeps a text line within a width limit.

    Parameters:
        text: Text to measure with OpenCV.
        max_width: Maximum permitted text width in pixels.
        font: OpenCV Hershey font identifier used for measurement.
        preferred_scale: Initial and largest desired font scale.
        min_scale: Smallest font scale that may be returned.
        thickness: Text stroke thickness in pixels.

    Returns:
        Largest tested font scale that fits within ``max_width``, or
        ``min_scale`` when the text cannot be reduced further.
    """
    scale = preferred_scale

    while scale > min_scale:
        (text_width, _), _ = cv2.getTextSize(text, font, scale, thickness)

        if text_width <= max_width:
            break

        scale -= 0.02

    return max(scale, min_scale)


def find_hotspot(img):
    """Detect the strongest LED-like hotspot in an image.

    The detector combines brightness and saturation thresholds, removes small
    mask artifacts, filters connected components by area, scores the remaining
    candidates, and calculates a brightness-weighted centroid for the best one.

    Parameters:
        img: BGR image as a NumPy array with shape ``(height, width, 3)``.
            The image may be a full camera frame or a cropped tracking ROI.

    Returns:
        A tuple ``(measurement, confidence, mask)`` where:

        measurement:
            A ``2 x 1`` float32 array ``[[x], [y]]`` containing the weighted
            hotspot centroid in coordinates local to ``img``. ``None`` when no
            acceptable candidate is found.
        confidence:
            Floating-point score of the selected candidate. Returns ``0.0``
            when no candidate is accepted.
        mask:
            Binary uint8 candidate mask used during connected-component
            analysis. May be ``None`` when the input is missing or too dark.
    """
    if img is None:
        return None, 0.0, None

    height, width = img.shape[:2]
    total_pixels = height * width

    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1].astype(np.float32)

    mean_gray = float(np.mean(gray))
    max_gray = float(np.max(gray))

    if max_gray < MIN_BRIGHTNESS:
        return None, 0.0, None

    percentile_99 = float(np.percentile(gray, 99.0))
    brightness_threshold = max(
        MIN_BRIGHTNESS,
        percentile_99,
        mean_gray + 35.0,
    )

    bright_mask = gray >= brightness_threshold
    saturated_mask = saturation >= MIN_SATURATION
    candidate_mask = bright_mask | (
        (gray >= mean_gray + 25.0) & saturated_mask
    )

    mask_uint8 = candidate_mask.astype(np.uint8) * 255
    mask_uint8 = cv2.morphologyEx(
        mask_uint8,
        cv2.MORPH_OPEN,
        np.ones((3, 3), dtype=np.uint8),
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

        if area > MAX_HOTSPOT_AREA_FRACTION * total_pixels:
            continue

        component_width = stats[label, cv2.CC_STAT_WIDTH]
        component_height = stats[label, cv2.CC_STAT_HEIGHT]
        component_mask = labels == label

        mean_hot = float(np.mean(gray[component_mask]))
        max_hot = float(np.max(gray[component_mask]))
        mean_saturation = float(np.mean(saturation[component_mask]))

        contrast = mean_hot / (mean_gray + 1e-6)
        compactness_bonus = 1.0 / max(1.0, np.sqrt(area))
        brightness_score = max_hot / 255.0
        contrast_score = max(0.5, contrast)
        saturation_score = 1.0 + (mean_saturation / 255.0)

        score = (
            brightness_score
            * contrast_score
            * saturation_score
            * min(3.0, area / 8.0)
            * (1.0 + compactness_bonus)
        )

        if component_width > 0.25 * width or component_height > 0.25 * height:
            score *= 0.2

        if score > best_score:
            best_score = score
            best_label = label

    if best_label is None:
        return None, 0.0, mask_uint8

    best_component_mask = labels == best_label
    ys, xs = np.where(best_component_mask)

    if len(xs) == 0:
        return None, 0.0, mask_uint8

    weights = gray[ys, xs]
    weights = weights / (np.sum(weights) + 1e-6)

    center_x = float(np.sum(xs * weights))
    center_y = float(np.sum(ys * weights))
    measurement = np.array([[center_x], [center_y]], dtype=np.float32)

    return measurement, float(best_score), mask_uint8


class HotspotDetector(Node):
    """ROS 2 node that detects, tracks, and publishes a bright hotspot."""

    def __init__(self):
        """Initialize ROS interfaces, Kalman tracking, and detector state.

        Parameters:
            None. Node settings and topic names are currently defined directly
            in this constructor and by the module-level configuration values.

        Returns:
            None.
        """
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
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32)
        self.kalman.statePost = np.zeros((4, 1), dtype=np.float32)

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
        self.get_logger().info(
            "Annotated image topic: /hotspot/annotated_image/compressed"
        )
        self.get_logger().info(
            f"Annotated image published every {ANNOTATION_EVERY_N_FRAMES} frames"
        )

    def image_callback(self, msg):
        """Decode an incoming compressed frame and pass it to the detector.

        Parameters:
            msg: ROS ``sensor_msgs/CompressedImage`` message. Its byte payload
                is decoded as a BGR image, and its header is retained for the
                corresponding annotated output message.

        Returns:
            None. Invalid image payloads are ignored.
        """
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            return

        self.frame_count += 1
        self.process_image(img, msg)

    def publish_target(self):
        """Publish the current visibility, normalized error, and confidence.

        Parameters:
            None. Values are read from the node's current tracking state.

        Returns:
            None. Publishes a ``std_msgs/Float32MultiArray`` on
            ``/hotspot/target``.
        """
        msg = Float32MultiArray()
        visible = self.target_visible and self.current_detection_valid

        msg.data = [
            1.0 if visible else 0.0,
            float(self.current_error_x),
            float(self.current_error_y),
            float(self.current_conf),
        ]
        self.target_pub.publish(msg)

    def process_image(self, img, input_msg):
        """Detect and track the hotspot in one decoded camera frame.

        Parameters:
            img: Current BGR camera frame as a NumPy array with shape
                ``(height, width, 3)``.
            input_msg: Original ``sensor_msgs/CompressedImage`` message. It is
                passed through so an annotated image can reuse its ROS header.

        Returns:
            None. Updates the Kalman filter and tracking state, publishes the
            target array, and periodically publishes an annotated image.
        """
        height, width = img.shape[:2]
        roi_rect = None

        if self.last_valid is not None and self.lost_counter < LOST_FRAMES_RESET:
            x, y = self.last_valid
            roi_side = max(2, int(round(width * ROI_WIDTH_FRACTION)))
            roi_half = max(1, roi_side // 2)

            x1 = max(0, x - roi_half)
            y1 = max(0, y - roi_half)
            x2 = min(width, x + roi_half)
            y2 = min(height, y + roi_half)

            search_img = img[y1:y2, x1:x2]
            offset = np.array([x1, y1], dtype=np.float32)
            self.mode = "ROI"
            roi_rect = (x1, y1, x2, y2)
        else:
            search_img = img
            offset = np.zeros(2, dtype=np.float32)
            self.mode = "FULL"

        self.kalman.predict()

        measurement, confidence, _ = find_hotspot(search_img)
        self.current_conf = float(confidence)

        valid = measurement is not None and confidence >= CONF_THRESHOLD
        self.current_detection_valid = valid

        if valid:
            measurement = np.array(measurement, dtype=np.float32, copy=True)
            measurement[:, 0] += offset

            noise = np.float32(max(0.05, 2.0 / (confidence + 1e-3)))
            self.kalman.measurementNoiseCov = (
                np.eye(2, dtype=np.float32) * noise
            )
            self.kalman.correct(measurement)

            self.last_valid = (
                int(measurement[0, 0]),
                int(measurement[1, 0]),
            )
            self.lost_counter = 0
            self.target_visible = True
        else:
            self.lost_counter += 1

            if self.lost_counter > LOST_FRAMES_RESET:
                self.target_visible = False
                self.current_error_x = 0.0
                self.current_error_y = 0.0

        estimate = self.kalman.statePost.ravel().astype(np.float32)

        if self.target_visible:
            estimated_x = float(estimate[0])
            estimated_y = float(estimate[1])

            self.current_error_x = (estimated_x - width * 0.5) / (width * 0.5)
            self.current_error_y = (estimated_y - height * 0.5) / (height * 0.5)

            self.traj.append((int(estimated_x), int(estimated_y)))
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
        """Create a visualization of the current detector and tracker state.

        Parameters:
            img: Source BGR image to copy and annotate.
            roi_rect: Optional ROI rectangle ``(x1, y1, x2, y2)`` in full-image
                pixel coordinates. ``None`` means no ROI rectangle is drawn.
            valid: Whether the current frame produced a detection whose score
                met ``CONF_THRESHOLD``.

        Returns:
            Annotated BGR image as a new NumPy array. The input image is not
            modified.
        """
        height, width = img.shape[:2]
        vis = img.copy()
        image_center = (width // 2, height // 2)

        for previous_point, current_point in zip(self.traj, self.traj[1:]):
            cv2.line(vis, previous_point, current_point, (255, 0, 0), 2)

        if roi_rect is not None:
            x1, y1, x2, y2 = roi_rect
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)

        target_point = (
            self.last_valid
            if self.target_visible and self.last_valid is not None
            else image_center
        )

        if valid:
            color = (0, 0, 255)
            status = "LED HOTSPOT"
        elif self.target_visible:
            color = (0, 165, 255)
            status = "TRACKING PREDICTED"
        else:
            color = (100, 100, 100)
            status = "NO LED"

        cv2.circle(vis, target_point, 8, color, -1)
        cv2.circle(vis, image_center, 8, (0, 255, 0), 2)

        if self.last_valid is not None:
            cv2.line(vis, image_center, self.last_valid, (255, 255, 0), 2)

        status_text = (
            f"{status} mode={self.mode} "
            f"conf={self.current_conf:.2f} "
            f"lost={self.lost_counter}"
        )
        error_text = (
            f"err=({self.current_error_x:+.2f}, {self.current_error_y:+.2f}) "
            f"valid={self.current_detection_valid}"
        )

        text_margin = 10
        max_text_width = max(1, width - 2 * text_margin)
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
            status_text,
            font,
            status_scale,
            text_thickness,
        )
        (_, error_height), error_baseline = cv2.getTextSize(
            error_text,
            font,
            error_scale,
            text_thickness,
        )

        status_y = text_margin + status_height
        error_y = status_y + status_baseline + error_height + 8
        panel_bottom = min(height, error_y + error_baseline + 6)

        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (width, panel_bottom), (0, 0, 0), -1)
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
        """JPEG-encode and publish an annotated image.

        Parameters:
            img: Annotated BGR image as a NumPy array.
            input_msg: Original ``sensor_msgs/CompressedImage`` message whose
                header is copied to the output message.

        Returns:
            None. Logs a warning and publishes nothing if JPEG encoding fails.
        """
        ok, encoded = cv2.imencode(
            ".jpg",
            img,
            [int(cv2.IMWRITE_JPEG_QUALITY), 75],
        )

        if not ok:
            self.get_logger().warn("Could not encode annotated image")
            return

        output_msg = CompressedImage()
        output_msg.header = input_msg.header
        output_msg.format = "jpeg"
        output_msg.data = encoded.tobytes()

        self.annotated_pub.publish(output_msg)


def main(args=None):
    """Start the ROS 2 node and process callbacks until shutdown.

    Parameters:
        args: Optional ROS 2 command-line arguments passed to ``rclpy.init``.
            Use ``None`` to let rclpy read the process command line normally.

    Returns:
        None. The function blocks in ``rclpy.spin`` until interrupted, then
        destroys the node and shuts down rclpy cleanly.
    """
    rclpy.init(args=args)
    node = HotspotDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
