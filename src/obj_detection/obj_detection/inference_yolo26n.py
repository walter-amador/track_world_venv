#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

# Workspace root (src/obj_detection/obj_detection/ → up 3 levels)
_WS_ROOT = Path(__file__).resolve().parents[3]
MODEL_PATH = str(_WS_ROOT / "models" / "yolo26n_openvino_model")

# Robot camera: 70° HFOV, 640×480 (CLAUDE.md spec → focal ≈ 457 px)
_FOCAL_PX = 640.0 / (2.0 * np.tan(np.radians(35.0)))


def _imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to a BGR numpy array without cv_bridge."""
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding in ("rgb8", "RGB8"):
        return img[:, :, ::-1].copy()
    return img  # bgr8 or mono — pass through


def estimate_distance(pixel_size: float, real_size_cm: float = 8.0) -> float | None:
    """Pinhole estimate: D = f * real_size / pixel_size."""
    if pixel_size > 0:
        return (_FOCAL_PX * real_size_cm) / pixel_size
    return None


def draw_detections(frame, results, class_names):
    closest = {"class_name": None, "distance_cm": 0.0}

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            confidence = float(box.conf[0])
            class_name = class_names[int(box.cls[0])]

            box_size = max(x2 - x1, y2 - y1)
            distance_cm = estimate_distance(box_size)

            if distance_cm and (
                distance_cm < closest["distance_cm"] or closest["distance_cm"] == 0.0
            ):
                closest = {"class_name": class_name, "distance_cm": distance_cm}

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = (
                f"{class_name}: {confidence:.2f} | {distance_cm:.1f} cm"
                if distance_cm
                else f"{class_name}: {confidence:.2f}"
            )
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - baseline - 5), (x1 + tw, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    return frame, closest


class YoloDetectionNode(Node):
    def __init__(self, conf: float, iou: float):
        super().__init__("yolo26n_detection")

        self._conf = conf
        self._iou = iou
        self._prev_time = time.time()
        self._window = "YOLO26n — Traffic Sign Detection [OpenVINO]"

        if not Path(MODEL_PATH).exists():
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            raise RuntimeError(f"Model not found: {MODEL_PATH}")

        self.get_logger().info(f"Loading {MODEL_PATH} ...")
        self._model = YOLO(MODEL_PATH, task="detect")
        self._class_names = self._model.names
        self.get_logger().info(f"Classes: {list(self._class_names.values())}")
        self.get_logger().info(f"conf={conf:.2f}  iou={iou:.2f}  focal={_FOCAL_PX:.0f} px")

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        self.get_logger().info("Subscribing to /camera/image_raw — press 'q' in window to quit")

    def _on_image(self, msg: Image):
        frame = _imgmsg_to_bgr(msg)

        results = self._model(frame, conf=self._conf, iou=self._iou, verbose=False)
        frame, closest = draw_detections(frame, results, self._class_names)

        now = time.time()
        fps = 1.0 / max(now - self._prev_time, 1e-9)
        self._prev_time = now

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "Backend: OpenVINO", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)

        if closest["class_name"]:
            cv2.putText(
                frame,
                f"Closest: {closest['class_name']}  {closest['distance_cm']:.1f} cm",
                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )

        cv2.imshow(self._window, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("'q' pressed — shutting down")
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description="YOLO26n traffic-sign inference on the robot's ROS2 camera"
    )
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--iou",  type=float, default=0.45, help="IoU threshold")
    args, _ = parser.parse_known_args()  # ignore ROS2 remapping args

    rclpy.init()
    node = YoloDetectionNode(conf=args.conf, iou=args.iou)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped.")


if __name__ == "__main__":
    main()
