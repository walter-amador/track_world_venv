"""
depth_benchmark_node.py — passive depth estimation benchmark.

Subscribes to /camera/image_raw, runs the selected depth model, and
publishes a colourised depth map to /depth/colorized for visual inspection
in rqt_image_view.  You drive the robot via teleop; this node only watches.

Published topics:
  /depth/colorized  (sensor_msgs/Image, BGR8)  — jet-colourised depth map
                                                  overlaid with obstacle zone

Logged per-frame:
  - inference latency (ms)
  - rolling 10-frame FPS average

Usage:
  ros2 launch obst_avoid depth_benchmark.launch.py model:=midas
  ros2 launch obst_avoid depth_benchmark.launch.py model:=depth_anything_v2
  ros2 launch obst_avoid depth_benchmark.launch.py model:=zoe_depth
  ros2 launch obst_avoid depth_benchmark.launch.py model:=depth_pro

View the output:
  ros2 run rqt_image_view rqt_image_view /depth/colorized
"""

import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from obst_avoid.model_loader import build_model


class DepthBenchmarkNode(Node):

    def __init__(self):
        super().__init__("depth_benchmark_node")

        # ── parameters ─────────────────────────────────────────────────────
        self.declare_parameter("model_name", "midas")
        self.declare_parameter("model_variant", "small")
        # Obstacle zone: centre strip [left_frac, right_frac] of frame width
        self.declare_parameter("zone_left_frac", 0.33)
        self.declare_parameter("zone_right_frac", 0.67)
        # Normalised depth [0=closest, 1=farthest]; values below this are highlighted
        self.declare_parameter("obstacle_threshold", 0.20)

        model_name = self.get_parameter("model_name").value
        model_variant = self.get_parameter("model_variant").value
        self._zone_l = self.get_parameter("zone_left_frac").value
        self._zone_r = self.get_parameter("zone_right_frac").value
        self._threshold = self.get_parameter("obstacle_threshold").value

        # ── model ───────────────────────────────────────────────────────────
        self.get_logger().info(
            f"Loading model '{model_name}' (variant='{model_variant}')…"
            " This may take a minute on first run."
        )
        self._model = build_model(model_name, model_variant)
        self._model.load()
        self.get_logger().info(f"Model '{model_name}' ready.")

        # ── ROS I/O ─────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._sub = self.create_subscription(
            Image, "/camera/image_raw", self._image_cb, 1
        )
        self._pub = self.create_publisher(Image, "/depth/colorized", 1)

        self._latencies: deque = deque(maxlen=10)

    # ────────────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        t0 = time.monotonic()
        raw = self._model.infer(bgr)
        latency_ms = (time.monotonic() - t0) * 1000.0

        self._latencies.append(latency_ms)
        fps = 1000.0 / (sum(self._latencies) / len(self._latencies))
        self.get_logger().info(
            f"latency={latency_ms:.1f} ms  fps≈{fps:.1f}",
            throttle_duration_sec=1.0,
        )

        norm = self._model.to_normalized(raw)
        vis = self._make_vis(bgr, norm)

        out_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = msg.header
        self._pub.publish(out_msg)

    def _make_vis(self, bgr: np.ndarray, norm: np.ndarray) -> np.ndarray:
        """Jet-colourised depth + semi-transparent obstacle highlight."""
        h, w = norm.shape
        depth_uint8 = (norm * 255).astype(np.uint8)
        colourised = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)

        # Obstacle zone columns
        col_l = int(w * self._zone_l)
        col_r = int(w * self._zone_r)

        # Highlight pixels inside zone that are below threshold
        zone_norm = norm[:, col_l:col_r]
        obstacle_mask = (zone_norm < self._threshold).astype(np.uint8) * 255
        # Red overlay in zone
        overlay = colourised.copy()
        overlay[:, col_l:col_r][obstacle_mask > 0] = (0, 0, 255)
        vis = cv2.addWeighted(colourised, 0.7, overlay, 0.3, 0)

        # Zone boundary lines
        cv2.line(vis, (col_l, 0), (col_l, h), (255, 255, 255), 1)
        cv2.line(vis, (col_r, 0), (col_r, h), (255, 255, 255), 1)

        # Obstacle flag text
        zone_min = float(norm[:, col_l:col_r].min())
        label = f"zone_min={zone_min:.2f}  thr={self._threshold:.2f}"
        colour = (0, 0, 255) if zone_min < self._threshold else (0, 255, 0)
        cv2.putText(
            vis, label, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2, cv2.LINE_AA
        )
        model_label = f"model: {self._model.name}"
        cv2.putText(
            vis, model_label, (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA
        )
        return vis


def main(args=None):
    rclpy.init(args=args)
    node = DepthBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
