"""
obstacle_avoidance_node.py — depth-based obstacle avoidance PoC.

State machine:
  DRIVING      → forward at base_speed; checks centre zone for obstacles
  STOPPING     → zero velocity for stop_duration seconds (settle)
  TURNING      → turn right: linear.x=turn_speed, angular.z=-turn_angular_z
                 for turn_duration seconds → ≈90° right arc
  RESUMING     → brief straight burst before returning to DRIVING

Published topics:
  /cmd_vel          (geometry_msgs/Twist)  — suppressed when dry_run:=true
  /depth/colorized  (sensor_msgs/Image, BGR8) — debug view

Parameters (set via config/params.yaml or launch args):
  model_name          midas | depth_anything_v2 | zoe_depth | depth_pro
  model_variant       small | base | large | n | …
  obstacle_threshold  normalised depth [0,1]; below this triggers avoidance
  zone_left_frac      centre-zone left boundary (fraction of frame width)
  zone_right_frac     centre-zone right boundary
  zone_top_frac       centre-zone top boundary (fraction of frame height; 0=top)
  base_speed          forward speed in DRIVING (m/s)
  turn_speed          forward speed while turning (m/s)
  turn_angular_z      |angular.z| while turning; right turn → negative sent
  turn_duration       seconds to turn (tune to achieve ≈90° in your robot)
  stop_duration       seconds in STOPPING state before turning
  dry_run             true → run detection and show debug image, never move robot

Usage:
  # Autonomous (robot drives itself):
  ros2 launch obst_avoid obstacle_avoidance.launch.py model:=midas

  # Passive camera test (you drive with teleop, see what the node detects):
  ros2 launch obst_avoid obstacle_avoidance.launch.py model:=midas dry_run:=true
"""

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from obst_avoid.model_loader import build_model


class State:
    DRIVING = "DRIVING"
    STOPPING = "STOPPING"
    TURNING = "TURNING"
    RESUMING = "RESUMING"


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__("obstacle_avoidance_node")

        # ── parameters ─────────────────────────────────────────────────────
        self.declare_parameter("model_name", "midas")
        self.declare_parameter("model_variant", "small")
        self.declare_parameter("obstacle_threshold", 0.20)
        self.declare_parameter("zone_left_frac", 0.33)
        self.declare_parameter("zone_right_frac", 0.67)
        self.declare_parameter("zone_top_frac", 0.40)
        self.declare_parameter("base_speed", 0.20)
        self.declare_parameter("turn_speed", 0.10)
        self.declare_parameter("turn_angular_z", 0.45)
        self.declare_parameter("turn_duration", 3.5)
        self.declare_parameter("stop_duration", 0.5)
        self.declare_parameter("resume_duration", 0.5)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("dry_run", False)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        model_name = p("model_name")
        model_variant = p("model_variant")
        self._threshold = p("obstacle_threshold")
        self._zone_l = p("zone_left_frac")
        self._zone_r = p("zone_right_frac")
        self._zone_top = p("zone_top_frac")
        self._base_speed = p("base_speed")
        self._turn_speed = p("turn_speed")
        self._turn_az = p("turn_angular_z")
        self._turn_dur = p("turn_duration")
        self._stop_dur = p("stop_duration")
        self._resume_dur = p("resume_duration")
        self._publish_debug = p("publish_debug")
        self._dry_run = p("dry_run")

        # ── model ───────────────────────────────────────────────────────────
        self.get_logger().info(
            f"Loading model '{model_name}' (variant='{model_variant}')…"
        )
        self._model = build_model(model_name, model_variant)
        self._model.load()
        mode = "DRY RUN (no /cmd_vel)" if self._dry_run else "AUTONOMOUS"
        self.get_logger().info(f"Model '{model_name}' ready. Mode: {mode}.")

        # ── state machine ───────────────────────────────────────────────────
        self._state = State.DRIVING
        self._state_entered_at: float = time.monotonic()

        # ── ROS I/O ─────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._sub = self.create_subscription(
            Image, "/camera/image_raw", self._image_cb, 1
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        if self._publish_debug:
            self._dbg_pub = self.create_publisher(Image, "/depth/colorized", 1)

        # Timer drives state transitions that don't need a new camera frame
        # (e.g., stopping → turning → resuming)
        self._timer = self.create_timer(0.05, self._timer_cb)

    # ── state machine ────────────────────────────────────────────────────────

    def _enter(self, new_state: str) -> None:
        self.get_logger().info(f"{self._state} → {new_state}")
        self._state = new_state
        self._state_entered_at = time.monotonic()

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_entered_at

    def _timer_cb(self) -> None:
        if self._dry_run:
            return

        if self._state == State.STOPPING and self._elapsed() >= self._stop_dur:
            self._enter(State.TURNING)
        elif self._state == State.TURNING and self._elapsed() >= self._turn_dur:
            self._enter(State.RESUMING)
        elif self._state == State.RESUMING and self._elapsed() >= self._resume_dur:
            self._enter(State.DRIVING)

        # Continuously send the correct velocity for non-DRIVING states
        if self._state == State.STOPPING:
            self._publish_vel(0.0, 0.0)
        elif self._state == State.TURNING:
            # Positive turn_speed forward + negative angular.z → right arc
            self._publish_vel(self._turn_speed, -self._turn_az)
        elif self._state == State.RESUMING:
            self._publish_vel(self._base_speed, 0.0)

    # ── camera callback ──────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        raw = self._model.infer(bgr)
        norm = self._model.to_normalized(raw)

        obstacle = self._check_obstacle(norm)

        if self._dry_run:
            if obstacle:
                self.get_logger().warn(
                    "DRY RUN — obstacle detected (would trigger avoidance).",
                    throttle_duration_sec=1.0,
                )
        elif self._state == State.DRIVING:
            if obstacle:
                self.get_logger().warn("Obstacle detected — initiating avoidance.")
                self._enter(State.STOPPING)
            else:
                self._publish_vel(self._base_speed, 0.0)

        if self._publish_debug:
            vis = self._make_vis(bgr, norm, obstacle)
            out_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            out_msg.header = msg.header
            self._dbg_pub.publish(out_msg)

    def _check_obstacle(self, norm: np.ndarray) -> bool:
        """True if the 10th-percentile normalised depth in the centre zone < threshold."""
        h, w = norm.shape
        row_top = int(h * self._zone_top)
        col_l = int(w * self._zone_l)
        col_r = int(w * self._zone_r)
        zone = norm[row_top:, col_l:col_r]
        if zone.size == 0:
            return False
        # 10th percentile: robust to noise / specular spots
        p10 = float(np.percentile(zone, 10))
        return p10 < self._threshold

    # ── helpers ──────────────────────────────────────────────────────────────

    def _publish_vel(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _make_vis(
        self, bgr: np.ndarray, norm: np.ndarray, obstacle: bool
    ) -> np.ndarray:
        h, w = norm.shape
        depth_uint8 = (norm * 255).astype(np.uint8)
        vis = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)

        col_l = int(w * self._zone_l)
        col_r = int(w * self._zone_r)
        row_top = int(h * self._zone_top)

        # Zone rectangle
        box_colour = (0, 0, 255) if obstacle else (0, 255, 0)
        cv2.rectangle(vis, (col_l, row_top), (col_r, h - 1), box_colour, 2)

        # State + mode label
        if self._dry_run:
            state_text = f"DRY RUN — {'OBSTACLE' if obstacle else 'clear'}"
            state_colour = (0, 0, 255) if obstacle else (0, 255, 0)
        else:
            state_text = f"STATE: {self._state}"
            state_colour = (0, 0, 255) if self._state != State.DRIVING else (0, 255, 0)
        cv2.putText(
            vis, state_text, (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, state_colour, 2, cv2.LINE_AA
        )
        cv2.putText(
            vis, f"model: {self._model.name}", (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA
        )
        return vis


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
