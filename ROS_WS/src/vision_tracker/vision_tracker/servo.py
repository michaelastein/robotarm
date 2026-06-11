import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
import threading
import sys

from moveit.planning import MoveItPy


CONF_THRESHOLD = 5.0


# ---------------- HOTSPOT DETECTION ----------------
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


# ---------------- NODE ----------------
class VisionMoveItServo(Node):

    def __init__(self):
        super().__init__('vision_moveit_servo')

        # camera
        self.sub = self.create_subscription(
            CompressedImage,
            '/cam0/camera/image_raw/compressed',
            self.callback,
            qos_profile_sensor_data
        )

        # state
        self.lock = threading.Lock()
        self.latest = None

        # MoveIt
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

        self.x = 0.0
        self.y = 0.0
        self.conf = 0.0

        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("ONE-SCRIPT vision servo started")

    def callback(self, msg):
        with self.lock:
            self.latest = msg

    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):

        # get image
        with self.lock:
            msg = self.latest
            self.latest = None

        if msg is None:
            return

        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return

        h, w = img.shape[:2]

        meas, conf = find_hotspot(img)

        if meas is None or conf < CONF_THRESHOLD:
            return

        x, y = meas

        # error normalized [-1,1]
        ex = (x - w * 0.5) / (w * 0.5)
        ey = (y - h * 0.5) / (h * 0.5)

        self.get_logger().info(f"error x={ex:.3f} y={ey:.3f} conf={conf:.2f}")

        if abs(ex) < 0.05 and abs(ey) < 0.05:
            return

        # ---------------- GET CURRENT POSE ----------------
        state = self.moveit.get_current_state()
        pose = state.get_global_link_transform("end_effector_link")

        gain = 0.05

        # camera-frame correction
        pose.translation.x += ex * gain
        pose.translation.y += ey * gain

        # ---------------- SEND TO MOVEIT ----------------
        self.arm.set_goal_state(pose_stamped_msg=pose)

        plan = self.arm.plan()

        if plan:
            self.arm.execute()


# ---------------- MAIN ----------------
def main():
    rclpy.init()
    node = VisionMoveItServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
