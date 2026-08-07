#!/usr/bin/env python3
"""
ROS 2 MJPEG web-stream node for hotspot camera images.

This node exposes a browser-compatible MJPEG stream on an HTTP server. It
subscribes to both the raw camera stream and the annotated hotspot stream.

Stream selection behavior:
    - Raw images are used until annotated images become available.
    - Annotated images take priority while they continue arriving.
    - If annotated images stop for longer than ANNOTATED_TIMEOUT_SECONDS, the
      node immediately falls back to the most recently received raw frame.

ROS topics:
    RAW_IMAGE_TOPIC:
        sensor_msgs/msg/CompressedImage containing the raw JPEG camera frames.

    ANNOTATED_IMAGE_TOPIC:
        sensor_msgs/msg/CompressedImage containing hotspot annotations.

HTTP endpoints:
    /:
        Minimal full-window HTML page displaying the MJPEG stream.

    /stream.mjpg:
        Multipart MJPEG stream suitable for a browser or video client.

Configuration parameters:
    HTTP_HOST:
        Network interface on which the HTTP server listens. "0.0.0.0" allows
        access through any network interface on the computer.

    HTTP_PORT:
        TCP port used by the web server. Open the stream in a browser using
        http://<computer-ip>:HTTP_PORT.

    ANNOTATED_TIMEOUT_SECONDS:
        Maximum time, in seconds, since the latest annotated frame before the
        stream falls back to the raw camera image.

    FALLBACK_CHECK_PERIOD_SECONDS:
        Period, in seconds, at which the ROS timer checks whether the annotated
        stream has timed out.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


RAW_IMAGE_TOPIC = "/cam0/camera/image_raw/compressed"
ANNOTATED_IMAGE_TOPIC = "/hotspot/annotated_image/compressed"

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000

ANNOTATED_TIMEOUT_SECONDS = 1.0
FALLBACK_CHECK_PERIOD_SECONDS = 0.1


latest_frame: Optional[bytes] = None
latest_frame_id = 0
frame_condition = threading.Condition()


class HotspotImageWebNode(Node):
    """Select raw or annotated ROS camera frames for the web stream."""

    def __init__(self) -> None:
        """
        Initialize the ROS subscriptions and annotated-stream fallback timer.

        Parameters:
            None.

        Returns:
            None.
        """
        super().__init__("hotspot_web_stream")

        self.last_annotated_time: Optional[float] = None
        self.latest_raw_frame: Optional[bytes] = None
        self.using_annotated = False

        self.raw_sub = self.create_subscription(
            CompressedImage,
            RAW_IMAGE_TOPIC,
            self.raw_image_callback,
            qos_profile_sensor_data,
        )

        self.annotated_sub = self.create_subscription(
            CompressedImage,
            ANNOTATED_IMAGE_TOPIC,
            self.annotated_image_callback,
            qos_profile_sensor_data,
        )

        self.fallback_timer = self.create_timer(
            FALLBACK_CHECK_PERIOD_SECONDS,
            self.check_annotated_timeout,
        )

        self.get_logger().info(
            f"Camera stream available at http://{HTTP_HOST}:{HTTP_PORT}"
        )

    def publish_web_frame(self, frame: bytes) -> None:
        """
        Store a JPEG frame and notify all waiting HTTP stream clients.

        Parameters:
            frame:
                Complete JPEG image data as immutable bytes. The frame is
                forwarded without decoding or re-encoding.

        Returns:
            None.
        """
        global latest_frame, latest_frame_id

        with frame_condition:
            latest_frame = frame
            latest_frame_id += 1
            frame_condition.notify_all()

    def annotated_is_active(self) -> bool:
        """
        Check whether annotated frames have arrived recently enough.

        Parameters:
            None.

        Returns:
            True when an annotated frame was received within
            ANNOTATED_TIMEOUT_SECONDS; otherwise False.
        """
        if self.last_annotated_time is None:
            return False

        elapsed = time.monotonic() - self.last_annotated_time
        return elapsed <= ANNOTATED_TIMEOUT_SECONDS

    def raw_image_callback(self, msg: CompressedImage) -> None:
        """
        Receive and cache a raw camera frame.

        Raw frames are published to the web stream only when the annotated
        stream is unavailable or has timed out.

        Parameters:
            msg:
                ROS CompressedImage message containing the raw JPEG frame in
                msg.data.

        Returns:
            None.
        """
        frame = bytes(msg.data)
        self.latest_raw_frame = frame

        if self.annotated_is_active():
            return

        self.using_annotated = False
        self.publish_web_frame(frame)

    def annotated_image_callback(self, msg: CompressedImage) -> None:
        """
        Receive an annotated frame and give it priority over raw frames.

        Parameters:
            msg:
                ROS CompressedImage message containing the annotated JPEG
                frame in msg.data.

        Returns:
            None.
        """
        self.last_annotated_time = time.monotonic()
        self.using_annotated = True
        self.publish_web_frame(bytes(msg.data))

    def check_annotated_timeout(self) -> None:
        """
        Fall back to the latest raw frame when annotations stop arriving.

        Parameters:
            None.

        Returns:
            None.
        """
        if not self.using_annotated or self.annotated_is_active():
            return

        self.using_annotated = False

        if self.latest_raw_frame is not None:
            self.publish_web_frame(self.latest_raw_frame)


class MJPEGHandler(BaseHTTPRequestHandler):
    """Serve the HTML viewer and multipart MJPEG camera stream."""

    protocol_version = "HTTP/1.1"

    def do_GET(self) -> None:
        """
        Handle HTTP GET requests.

        Supported paths:
            /:
                Returns a minimal full-screen HTML camera viewer.

            /stream.mjpg:
                Returns a continuous multipart MJPEG stream.

            /favicon.ico:
                Returns an empty response to prevent unnecessary browser errors.

        Parameters:
            None. Request information is provided by BaseHTTPRequestHandler.

        Returns:
            None.
        """
        if self.path == "/":
            self._serve_index()
            return

        if self.path.startswith("/stream.mjpg"):
            self._serve_mjpeg_stream()
            return

        if self.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return

        self.send_error(404)

    def _serve_index(self) -> None:
        """
        Send the browser page that displays the MJPEG stream.

        Parameters:
            None.

        Returns:
            None.
        """
        html = b"""<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <title>Hotspot Camera</title>
        <style>
            html, body {
                margin: 0;
                width: 100%;
                height: 100%;
                background: #111;
                overflow: hidden;
            }

            img {
                display: block;
                width: 100vw;
                height: 100vh;
                object-fit: contain;
            }
        </style>
    </head>
    <body>
        <img src="/stream.mjpg" alt="Hotspot camera stream">
    </body>
</html>
"""

        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.send_header(
            "Cache-Control",
            "no-store, no-cache, must-revalidate, max-age=0",
        )
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(html)
        self.wfile.flush()

    def _serve_mjpeg_stream(self) -> None:
        """
        Stream each newly received JPEG frame to one HTTP client.

        The condition variable prevents unnecessary polling. Each connected
        client waits until the shared frame identifier changes, then receives
        exactly one new multipart JPEG section.

        Parameters:
            None.

        Returns:
            None. The method runs until the client disconnects.
        """
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
                            and latest_frame_id != last_sent_frame_id
                        )
                    )

                    frame = latest_frame
                    last_sent_frame_id = latest_frame_id

                if frame is None:
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, format: str, *args: object) -> None:
        """
        Disable BaseHTTPRequestHandler access-log output.

        Parameters:
            format:
                Format string generated by BaseHTTPRequestHandler.

            *args:
                Values associated with the access-log format string.

        Returns:
            None.
        """
        return


def start_http_server(
    host: str = HTTP_HOST,
    port: int = HTTP_PORT,
) -> None:
    """
    Create and run the threaded HTTP server.

    Parameters:
        host:
            Interface address on which the server listens. Use "0.0.0.0" to
            accept connections through all available interfaces.

        port:
            TCP port on which the HTTP server listens.

    Returns:
        None. This function blocks until the server is shut down.
    """
    server = ThreadingHTTPServer((host, port), MJPEGHandler)
    server.daemon_threads = True
    server.serve_forever()


def main(args=None) -> None:
    """
    Initialize ROS 2, start the HTTP server, and run the image node.

    Parameters:
        args:
            Optional command-line arguments passed to rclpy.init(). When None,
            rclpy uses the process command-line arguments.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = HotspotImageWebNode()

    http_thread = threading.Thread(
        target=start_http_server,
        kwargs={"host": HTTP_HOST, "port": HTTP_PORT},
        daemon=True,
        name="hotspot-mjpeg-server",
    )
    http_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()