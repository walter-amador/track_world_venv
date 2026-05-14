"""
Lane Controller Node
--------------------
A PID + curvature-feedforward controller that converts the normalised lateral
error and road curvature published by LaneDetectionNode into cmd_vel commands
for the LIMO in Ackermann mode.

Sign convention (right-lane following):
  error > 0  →  robot is left  of right-lane centre  →  steer right  (angular.z < 0)
  error < 0  →  robot is right of right-lane centre  →  steer left   (angular.z > 0)
  output      =  -(Kp·e + Ki·∫e·dt + Kd·ė) - Kff·curvature

Curvature feedforward:
  curvature from /lane/curvature = sum of the quadratic 'a' coefficients of the
  left and right polynomial fits (px⁻¹).  Positive = right curve, negative = left.
  Feedforward term: ff = -Kff_curvature * curvature  (eliminates steady-state
  lateral offset on curves; without it, Kp alone requires ~30 mm of persistent
  error to maintain the steering needed to follow a 1.5 m radius arc).

Speed management:
  Forward speed is reduced proportionally to steering magnitude, so the robot
  slows on sharp curves and maintains speed on straights.

Subscribed topics
  /lane/lateral_error   std_msgs/Float64  error from LaneDetectionNode
  /lane/curvature       std_msgs/Float64  road curvature (px⁻¹) from LaneDetectionNode
  /behavior/state       std_msgs/String   gate from BehaviorManagerNode

Published topics
  /cmd_vel              geometry_msgs/Twist

Parameters  (see config/params.yaml)
  Kp, Ki, Kd              PID gains
  Kff_curvature           curvature feedforward gain (steering rad / curvature px⁻¹)
  base_speed              m/s forward speed on straights
  max_angular_z           rad/s steering magnitude cap
  speed_reduction_factor  [0,1] fraction of speed cut at max steering
  control_rate            Hz
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist


class LaneControllerNode(Node):
    def __init__(self):
        super().__init__('lane_controller_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('Kp', 1.2)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.08)
        self.declare_parameter('Kff_curvature', 20.0)
        self.declare_parameter('base_speed', 0.3)           # m/s
        self.declare_parameter('max_angular_z', 0.5)        # rad/s
        self.declare_parameter('speed_reduction_factor', 0.5)
        self.declare_parameter('control_rate', 20.0)        # Hz

        # ── state ────────────────────────────────────────────────────────────
        self._error          = 0.0
        self._prev_error     = 0.0
        self._integral       = 0.0
        self._curvature      = 0.0
        self._behavior_state = 'STOP'   # safe default until manager sends state
        self._prev_time      = self.get_clock().now()

        # ── pub / sub ────────────────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64, '/lane/lateral_error', self._on_error,     10)
        self.create_subscription(Float64, '/lane/curvature',     self._on_curvature, 10)
        self.create_subscription(String,  '/behavior/state',     self._on_behavior,  10)

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._loop)

        self.get_logger().info('LaneControllerNode started')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_error(self, msg: Float64):
        self._error = msg.data

    def _on_curvature(self, msg: Float64):
        self._curvature = msg.data

    def _on_behavior(self, msg: String):
        self._behavior_state = msg.data

    # ── control loop ─────────────────────────────────────────────────────────

    def _loop(self):
        now = self.get_clock().now()
        dt  = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._prev_time = now

        cmd = Twist()

        if self._behavior_state == 'FOLLOW_LANE':
            Kp  = self.get_parameter('Kp').value
            Ki  = self.get_parameter('Ki').value
            Kd  = self.get_parameter('Kd').value
            Kff = self.get_parameter('Kff_curvature').value
            v0  = self.get_parameter('base_speed').value
            cap = self.get_parameter('max_angular_z').value
            srf = self.get_parameter('speed_reduction_factor').value

            e = self._error

            self._integral  += e * dt
            # Anti-windup: clamp integral contribution to ±1 equivalent steering
            self._integral   = max(-cap / max(Ki, 1e-9),
                                   min(cap / max(Ki, 1e-9), self._integral)) \
                                if Ki > 1e-9 else 0.0

            derivative       = (e - self._prev_error) / dt
            self._prev_error = e

            # PID term: negative sign → positive error (left of lane) → steer right
            # Curvature feedforward: -Kff * curvature anticipates curves without
            # waiting for lateral error to build up (positive curvature = right curve
            # → negative ff = steer right).
            pid_term  = -(Kp * e + Ki * self._integral + Kd * derivative)
            ff_term   = -Kff * self._curvature
            raw_steer = pid_term + ff_term
            steer     = max(-cap, min(cap, raw_steer))

            # Adaptive speed: slow down proportionally to steering effort
            speed = v0 * (1.0 - srf * abs(steer) / max(cap, 1e-9))
            speed = max(0.05, speed)

            cmd.linear.x  = speed
            cmd.angular.z = steer

        elif self._behavior_state == 'RECOVER':
            # Creep forward with gentler last-known correction
            Kp  = self.get_parameter('Kp').value
            cap = self.get_parameter('max_angular_z').value
            steer = max(-cap * 0.5,
                        min(cap * 0.5, -Kp * 0.4 * self._prev_error))
            cmd.linear.x  = 0.10
            cmd.angular.z = steer

        # All other states (STOP, APPROACH_INTERSECTION, …) → zero Twist
        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
