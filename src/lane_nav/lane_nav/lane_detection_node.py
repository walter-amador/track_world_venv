"""
Lane Detection Node
-------------------
Pipeline:
  1. Grayscale + Gaussian blur
  2. CLAHE  (adaptive histogram equalization — makes markings stand out locally
             regardless of global scene exposure or sun angle)
  3. Binary threshold via Otsu computed ONLY on ROI pixels
     (road trapezoid), then mask out everything outside the ROI
  4. Inverse Perspective Mapping  (trapezoid → bird's-eye rectangle)
  5. Histogram-seeded sliding windows  → pixel clouds per line
  6. 2nd-order polynomial fit  x = a·y² + b·y + c  per line
  7. Evaluate at look-ahead y  → right-lane-center x
  8. Lateral error  = (right_lane_cx − w/2) / (lane_half_width_px)
       positive → robot left of right-lane centre → steer right
       negative → robot right of right-lane centre → steer left

Debug image layout (640×480 output):
  ┌──────────────────────────────────────┐  ← 640×240
  │  raw camera + IPM trapezoid (red)    │
  │  Otsu thr=NNN                        │
  ├────────────────────┬─────────────────┤  ← each 320×240
  │  BINARY  (ROI-only)│  BIRD'S-EYE     │
  │  white px inside   │  + detections   │
  │  road region only  │  warped px count│
  └────────────────────┴─────────────────┘

Published topics
  /lane/lateral_error   std_msgs/Float64   [-1, 1]
  /lane/curvature       std_msgs/Float64   pixels⁻¹
  /lane/state           std_msgs/String    NORMAL|LOST_LEFT|LOST_RIGHT|LOST_BOTH
  /lane/debug_image     sensor_msgs/Image  (3-panel view)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.declare_parameter('ipm_src',
                               [80, 470, 560, 470, 420, 290, 220, 290])
        self.declare_parameter('ipm_dst_margin', 100)
        # Minimum Otsu threshold (safety floor for uniform/dark frames)
        self.declare_parameter('white_thresh', 80)
        self.declare_parameter('n_windows', 9)
        self.declare_parameter('window_margin', 50)
        self.declare_parameter('min_pix', 30)
        self.declare_parameter('lookahead_fraction', 0.3)
        self.declare_parameter('nominal_lane_half_px', 110)
        self.declare_parameter('smoothing_alpha', 0.75)
        self.declare_parameter('publish_debug', True)

        self._last_error  = 0.0
        self._roi_mask    = None   # computed on first frame

        self._pub_error = self.create_publisher(Float64, '/lane/lateral_error', 10)
        self._pub_curv  = self.create_publisher(Float64, '/lane/curvature',     10)
        self._pub_state = self.create_publisher(String,  '/lane/state',         10)
        self._pub_debug = self.create_publisher(Image,   '/lane/debug_image',   10)

        self._bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.get_logger().info('LaneDetectionNode started')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _build_roi_mask(self, h, w):
        """Fill the IPM source trapezoid with 255; everything else 0."""
        p   = self.get_parameter('ipm_src').value
        pts = np.array([[p[0],p[1]], [p[2],p[3]],
                        [p[4],p[5]], [p[6],p[7]]], dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        return mask

    def _ipm_matrix(self, w, h):
        p   = self.get_parameter('ipm_src').value
        src = np.float32([[p[0],p[1]], [p[2],p[3]],
                          [p[4],p[5]], [p[6],p[7]]])
        m   = self.get_parameter('ipm_dst_margin').value
        dst = np.float32([[m, h-1], [w-m, h-1], [w-m, 0], [m, 0]])
        return cv2.getPerspectiveTransform(src, dst)

    def _ipm_src_pts(self):
        p = self.get_parameter('ipm_src').value
        return np.array([[p[0],p[1]], [p[2],p[3]],
                         [p[4],p[5]], [p[6],p[7]]], dtype=np.int32)

    def _binarize(self, bgr, roi_mask):
        """
        CLAHE → Otsu (ROI-only) → ROI mask.

        CLAHE equalises contrast tile-by-tile so lane markings always appear
        brighter than the road even when global scene brightness varies.
        Otsu is then computed only on road-region pixels so sky/ground
        contamination cannot shift the threshold.
        The returned binary has non-zero pixels ONLY inside the road trapezoid.
        """
        gray    = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        clahe    = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(blurred)

        # Compute Otsu using only pixels inside the road ROI
        roi_px = enhanced[roi_mask > 0]
        min_thr = self.get_parameter('white_thresh').value
        if roi_px.size > 500:
            otsu_val, _ = cv2.threshold(
                roi_px.reshape(1, -1), 0, 255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            thr = max(int(otsu_val), min_thr)
        else:
            otsu_val = min_thr
            thr      = min_thr

        _, binary = cv2.threshold(enhanced, thr, 255, cv2.THRESH_BINARY)
        binary    = cv2.bitwise_and(binary, roi_mask)   # keep only road region

        return binary, int(otsu_val), gray, enhanced

    def _sliding_windows(self, warped):
        h, w    = warped.shape
        n       = self.get_parameter('n_windows').value
        margin  = self.get_parameter('window_margin').value
        min_pix = self.get_parameter('min_pix').value
        win_h   = h // n

        hist      = np.sum(warped[3*h//4:, :], axis=0)
        mid       = w // 2
        left_cur  = int(np.argmax(hist[:mid]))
        right_cur = int(np.argmax(hist[mid:]) + mid)

        nzy, nzx = warped.nonzero()
        left_ids, right_ids = [], []

        for i in range(n):
            y_lo = h - (i+1)*win_h
            y_hi = h - i*win_h
            lx_lo, lx_hi = left_cur  - margin, left_cur  + margin
            rx_lo, rx_hi = right_cur - margin, right_cur + margin

            good_l = np.where((nzy>=y_lo)&(nzy<y_hi)&(nzx>=lx_lo)&(nzx<lx_hi))[0]
            good_r = np.where((nzy>=y_lo)&(nzy<y_hi)&(nzx>=rx_lo)&(nzx<rx_hi))[0]
            left_ids.append(good_l)
            right_ids.append(good_r)
            if len(good_l) > min_pix: left_cur  = int(np.mean(nzx[good_l]))
            if len(good_r) > min_pix: right_cur = int(np.mean(nzx[good_r]))

        left_ids  = np.concatenate(left_ids)
        right_ids = np.concatenate(right_ids)
        min_total = min_pix * 3
        lx = nzx[left_ids]  if len(left_ids)  >= min_total else np.array([])
        ly = nzy[left_ids]  if len(left_ids)  >= min_total else np.array([])
        rx = nzx[right_ids] if len(right_ids) >= min_total else np.array([])
        ry = nzy[right_ids] if len(right_ids) >= min_total else np.array([])
        return lx, ly, rx, ry

    @staticmethod
    def _fit_poly(x, y):
        if len(x) < 5:
            return None
        try:
            return np.polyfit(y, x, 2)
        except (np.linalg.LinAlgError, ValueError):
            return None

    # ── main callback ────────────────────────────────────────────────────────

    def _on_image(self, msg):
        img  = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]

        # Build ROI mask once (params don't change at runtime)
        if self._roi_mask is None:
            self._roi_mask = self._build_roi_mask(h, w)
            self.get_logger().info(
                f'ROI mask built: {int(np.sum(self._roi_mask>0))} pixels inside trapezoid')

        binary, otsu_val, gray, enhanced = self._binarize(img, self._roi_mask)
        M      = self._ipm_matrix(w, h)
        warped = cv2.warpPerspective(binary, M, (w, h))

        warped_px = int(np.count_nonzero(warped))
        binary_px = int(np.count_nonzero(binary))

        # Log every ~30 frames so the terminal isn't flooded
        if not hasattr(self, '_log_ctr'):
            self._log_ctr = 0
        self._log_ctr += 1
        if self._log_ctr % 30 == 0:
            self.get_logger().info(
                f'thr={otsu_val}  binary_px={binary_px}  warped_px={warped_px}')

        lx, ly, rx, ry = self._sliding_windows(warped)
        lf = self._fit_poly(lx, ly)
        rf = self._fit_poly(rx, ry)

        laf   = self.get_parameter('lookahead_fraction').value
        y_eva = float(h - 1 - laf * (h - 1))
        nom   = float(self.get_parameter('nominal_lane_half_px').value)

        lane_state    = 'NORMAL'
        lateral_error = self._last_error
        curvature     = 0.0

        if lf is not None and rf is not None:
            lx_e = np.polyval(lf, y_eva)
            rx_e = np.polyval(rf, y_eva)
            if lx_e >= rx_e:
                lane_state = 'LOST_BOTH'
            else:
                half_w        = max((rx_e - lx_e) / 2.0, 30.0)
                lane_cx       = (lx_e + rx_e) / 2.0
                lateral_error = (lane_cx - w / 2.0) / half_w
                curvature     = float(lf[0] + rf[0])
                lane_state    = 'NORMAL'
        elif lf is not None:
            lx_e          = np.polyval(lf, y_eva)
            lane_cx       = lx_e + nom
            lateral_error = (lane_cx - w / 2.0) / nom
            curvature     = float(2.0 * lf[0])
            lane_state    = 'LOST_RIGHT'
        elif rf is not None:
            rx_e          = np.polyval(rf, y_eva)
            lane_cx       = rx_e - nom
            lateral_error = (lane_cx - w / 2.0) / nom
            curvature     = float(2.0 * rf[0])
            lane_state    = 'LOST_LEFT'
        else:
            lane_state = 'LOST_BOTH'

        lateral_error = float(np.clip(lateral_error, -1.0, 1.0))
        alpha = self.get_parameter('smoothing_alpha').value
        if lane_state != 'LOST_BOTH':
            lateral_error = alpha * lateral_error + (1.0 - alpha) * self._last_error
        self._last_error = lateral_error

        self._pub_error.publish(Float64(data=lateral_error))
        self._pub_curv.publish(Float64(data=curvature))
        state_msg = String(); state_msg.data = lane_state
        self._pub_state.publish(state_msg)

        if self.get_parameter('publish_debug').value:
            dbg = self._debug_image(
                img, binary, enhanced, warped,
                lx, ly, rx, ry, lf, rf,
                lateral_error, lane_state, y_eva,
                otsu_val, binary_px, warped_px, w, h)
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(dbg, 'bgr8'))

    # ── debug visualisation ──────────────────────────────────────────────────

    def _debug_image(self, raw, binary, enhanced, warped,
                     lx, ly, rx, ry, lf, rf,
                     error, state, y_eva,
                     otsu_val, binary_px, warped_px, w, h):
        half_h = h // 2    # 240
        half_w = w // 2    # 320

        sc = {
            'NORMAL':     (180, 255, 180),
            'LOST_RIGHT': (0, 200, 255),
            'LOST_LEFT':  (0, 200, 255),
            'LOST_BOTH':  (60, 60, 255),
        }.get(state, (255, 255, 255))

        # ── Panel A: raw camera + trapezoid + CLAHE-enhanced side-by-side ───
        raw_half      = cv2.resize(raw,      (half_w, half_h))
        enh_bgr       = cv2.cvtColor(cv2.resize(enhanced, (half_w, half_h)),
                                     cv2.COLOR_GRAY2BGR)
        panel_a = np.hstack([raw_half, enh_bgr])

        # Trapezoid on raw side (scale: x unchanged because half_w=320=w/2;
        # actually we need to scale x to half_w/w and y to half_h/h)
        pts = self._ipm_src_pts()
        pts_s = pts.copy().astype(float)
        pts_s[:, 0] = pts_s[:, 0] * half_w / w
        pts_s[:, 1] = pts_s[:, 1] * half_h / h
        cv2.polylines(panel_a, [pts_s.astype(np.int32)],
                      isClosed=True, color=(0, 50, 255), thickness=2)

        cv2.putText(panel_a, 'RAW',
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,50,255), 1)
        cv2.putText(panel_a, f'Otsu={otsu_val}',
                    (5, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,255), 1)
        cv2.putText(panel_a, 'CLAHE',
                    (half_w+5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

        # ── Panel B: ROI-masked binary ────────────────────────────────────────
        bin_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        panel_b = cv2.resize(bin_bgr, (half_w, half_h))
        cv2.putText(panel_b, 'BINARY (road ROI only)',
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100,255,100), 1)
        cv2.putText(panel_b, f'white px: {binary_px}',
                    (5, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100,255,100), 1)

        # ── Panel C: bird's-eye with detections ──────────────────────────────
        bev = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        if len(lx): bev[ly, lx] = (255, 140, 0)
        if len(rx): bev[ry, rx] = (0, 80, 255)
        ploty = np.linspace(0, h-1, h).astype(float)
        for coeff, col in [(lf, (0,220,255)), (rf, (0,255,180))]:
            if coeff is not None:
                fitx = np.polyval(coeff, ploty).astype(int)
                for yi, xi in zip(ploty.astype(int), fitx):
                    if 0 <= xi < w:
                        cv2.circle(bev, (xi, yi), 1, col, -1)
        cv2.line(bev, (0, int(y_eva)), (w, int(y_eva)), (160,160,160), 1)
        cx = int(w/2.0 + error*w/4.0)
        cv2.line(bev, (w//2, h-40), (w//2, h), (0,220,0),   2)
        cv2.line(bev, (cx,   h-40), (cx,   h), (0,165,255), 2)
        panel_c = cv2.resize(bev, (half_w, half_h))
        cv2.putText(panel_c, f'WARPED  px:{warped_px}',
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)
        cv2.putText(panel_c, state,
                    (5, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sc, 1)
        cv2.putText(panel_c, f'err={error:+.3f}',
                    (5, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sc, 1)

        # ── Assemble ──────────────────────────────────────────────────────────
        bottom = np.hstack([panel_b, panel_c])
        return np.vstack([panel_a, bottom])


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
