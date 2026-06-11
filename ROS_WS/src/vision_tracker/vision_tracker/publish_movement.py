import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import cv2
import numpy as np
import threading


CONF_THRESHOLD = 5.0


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
        (hot_mean / 255.0) *
        max(1.0, contrast) *
        (count / 10.0)
    )

    return np.array([cx, cy], dtype=np.float32), confidence


class VisionTracker(Node):

    def __init__(self):
        super().__init__('vision_tracker')

        self.sub = self.create_subscription(
            CompressedImage,
            '/cam0/camera/image_raw/compressed',
            self.callback,
            qos_profile_sensor_data
        )

        # Publishes normalized error [-1, 1]
        self.pub = self.create_publisher(Point, '/object_pixel', 10)

        self.lock = threading.Lock()
        self.latest_msg = None

        self.last_valid = None
        self.lost_counter = 0

        self.get_logger().info("Vision tracker started (NO MoveIt, vision only)")

    def callback(self, msg):
        with self.lock:
            self.latest_msg = msg


def main(args=None):

    rclpy.init(args=args)
    node = VisionTracker()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    while rclpy.ok():

        with node.lock:
            msg = node.latest_msg
            node.latest_msg = None

        if msg is None:
            cv2.waitKey(1)
            continue

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            continue

        h, w = img.shape[:2]

        meas, conf = find_hotspot(img)

        if meas is not None and conf >= CONF_THRESHOLD:

            x = meas[0]
            y = meas[1]

            # Normalize [-1, 1]
            nx = (x - (w * 0.5)) / (w * 0.5)
            ny = (y - (h * 0.5)) / (h * 0.5)

            node.last_valid = (int(x), int(y))
            node.lost_counter = 0

            out = Point()
            out.x = float(nx)
            out.y = float(ny)
            out.z = float(conf)

            node.pub.publish(out)

        else:
            node.lost_counter += 1

        # ---------------- DEBUG VIEW ----------------
        vis = img.copy()

        pt = node.last_valid if node.last_valid else (w // 2, h // 2)

        cv2.circle(vis, pt, 8, (0, 0, 255), -1)

        status = f"conf={conf:.2f} lost={node.lost_counter}"
        cv2.putText(vis, status, (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)

        cv2.imshow("Hotspot Tracker", vis)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
