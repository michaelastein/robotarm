#!/usr/bin/env python3

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


latest_frame = None
latest_frame_id = 0
frame_condition = threading.Condition()


class HotspotImageWebNode(Node):
    def __init__(self):
        super().__init__("hotspot_web_stream")

        self.annotated_timeout = 1.0
        self.last_annotated_time = None
        self.latest_raw_frame = None
        self.using_annotated = False
        self.received_first_frame = False

        self.raw_sub = self.create_subscription(
            CompressedImage,
            "/cam0/camera/image_raw/compressed",
            self.raw_image_callback,
            qos_profile_sensor_data,
        )

        self.annotated_sub = self.create_subscription(
            CompressedImage,
            "/hotspot/annotated_image/compressed",
            self.annotated_image_callback,
            qos_profile_sensor_data,
        )

        # Ensures fallback occurs even if the camera has a low frame rate.
        self.fallback_timer = self.create_timer(
            0.1,
            self.check_annotated_timeout,
        )

        self.get_logger().info(
            "Serving camera stream at http://0.0.0.0:8000"
        )
        self.get_logger().info(
            "Using raw camera until annotated images are available"
        )

    def publish_web_frame(self, frame):
        global latest_frame, latest_frame_id

        with frame_condition:
            latest_frame = frame
            latest_frame_id += 1
            frame_condition.notify_all()

        if not self.received_first_frame:
            self.received_first_frame = True
            self.get_logger().info("Received first image frame")

    def annotated_is_active(self):
        if self.last_annotated_time is None:
            return False

        return (
            time.monotonic() - self.last_annotated_time
            <= self.annotated_timeout
        )

    def raw_image_callback(self, msg):
        frame = bytes(msg.data)
        self.latest_raw_frame = frame

        # Ignore raw frames while annotated frames are arriving.
        if self.annotated_is_active():
            return

        if self.using_annotated:
            self.using_annotated = False
            self.get_logger().info(
                "Annotated stream stopped; switching to raw camera"
            )

        self.publish_web_frame(frame)

    def annotated_image_callback(self, msg):
        self.last_annotated_time = time.monotonic()

        if not self.using_annotated:
            self.using_annotated = True
            self.get_logger().info(
                "Annotated stream available; switching to annotated images"
            )

        self.publish_web_frame(bytes(msg.data))

    def check_annotated_timeout(self):
        if not self.using_annotated:
            return

        if self.annotated_is_active():
            return

        self.using_annotated = False
        self.get_logger().info(
            "Annotated stream timed out; switching to raw camera"
        )

        # Immediately show the most recently received raw frame.
        if self.latest_raw_frame is not None:
            self.publish_web_frame(self.latest_raw_frame)


class MJPEGHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if self.path == "/":
            html = """
            <!doctype html>
            <html>
                <head>
                    <title>Hotspot Camera</title>
                    <style>
                        html, body {
                            margin: 0;
                            padding: 0;
                            width: 100%;
                            height: 100%;
                            background: #111;
                            overflow: hidden;
                        }

                        img {
                            width: 100vw;
                            height: 100vh;
                            object-fit: contain;
                            display: block;
                        }
                    </style>
                </head>
                <body>
                    <img src="/stream.mjpg">
                </body>
            </html>
            """.encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "text/html; charset=utf-8",
            )
            self.send_header("Content-Length", str(len(html)))
            self.send_header(
                "Cache-Control",
                "no-store, no-cache, must-revalidate, max-age=0",
            )
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(html)
            self.wfile.flush()
            return

        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header(
                "Cache-Control",
                "no-store, no-cache, must-revalidate, max-age=0",
            )
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()

            last_sent_frame_id = -1

            try:
                while True:
                    with frame_condition:
                        frame_condition.wait_for(
                            lambda: (
                                latest_frame is not None
                                and latest_frame_id
                                != last_sent_frame_id
                            )
                        )

                        frame = latest_frame
                        last_sent_frame_id = latest_frame_id

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(
                        b"Content-Type: image/jpeg\r\n"
                    )
                    self.wfile.write(
                        (
                            f"Content-Length: {len(frame)}\r\n\r\n"
                        ).encode("utf-8")
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()

            except (
                BrokenPipeError,
                ConnectionResetError,
            ):
                pass
            except Exception as error:
                print(
                    "Stream client disconnected/error: "
                    f"{error}"
                )

            return

        if self.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return

        self.send_error(404)

    def log_message(self, format, *args):
        return


def start_http_server(host="0.0.0.0", port=8000):
    server = ThreadingHTTPServer(
        (host, port),
        MJPEGHandler,
    )
    server.daemon_threads = True
    server.serve_forever()


def main(args=None):
    rclpy.init(args=args)

    node = HotspotImageWebNode()

    http_thread = threading.Thread(
        target=start_http_server,
        kwargs={
            "host": "0.0.0.0",
            "port": 8000,
        },
        daemon=True,
    )
    http_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
