"""
Behavior Manager Node
---------------------
Central state machine that coordinates what the robot does.  It is the
integration point for all perception nodes:

  Currently wired:
    /lane/state        from LaneDetectionNode
    /behavior/command  from any external node (YOLO, manual, etc.)

  Future integration hooks (subscribe and add transitions):
    /sign/detections   YOLO traffic-sign detector output
    /crosswalk/state   crosswalk detector output
    /intersection/type intersection classifier output

States and their meanings
  FOLLOW_LANE          Normal lane following at full speed
  RECOVER              Lane lost — slow forward, try to reacquire (timeout → STOP)
  STOP                 Full stop  (triggered by command or recovery timeout)
  APPROACH_INTERSECTION Slow approach — future: triggered by sign detector
  TURN_LEFT            Intersection left-turn manoeuvre  — future
  TURN_RIGHT           Intersection right-turn manoeuvre — future
  GO_STRAIGHT          Intersection straight-through     — future
  WAIT_CROSSWALK       Yield at crosswalk                — future

External command interface  (/behavior/command  std_msgs/String):
  'START'        → FOLLOW_LANE
  'STOP'         → STOP
  'RECOVER'      → RECOVER
  'TURN_LEFT'    → TURN_LEFT   (no-op until intersection logic is implemented)
  'TURN_RIGHT'   → TURN_RIGHT
  'GO_STRAIGHT'  → GO_STRAIGHT
  'APPROACH'     → APPROACH_INTERSECTION

Published topics
  /behavior/state   std_msgs/String  (consumed by LaneControllerNode)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BehaviorManagerNode(Node):

    _ALL_STATES = frozenset({
        'FOLLOW_LANE', 'RECOVER', 'STOP',
        'APPROACH_INTERSECTION', 'TURN_LEFT', 'TURN_RIGHT',
        'GO_STRAIGHT', 'WAIT_CROSSWALK',
    })

    _CMD_MAP = {
        'START':    'FOLLOW_LANE',
        'STOP':     'STOP',
        'RECOVER':  'RECOVER',
        'APPROACH': 'APPROACH_INTERSECTION',
        'TURN_LEFT':    'TURN_LEFT',
        'TURN_RIGHT':   'TURN_RIGHT',
        'GO_STRAIGHT':  'GO_STRAIGHT',
    }

    def __init__(self):
        super().__init__('behavior_manager_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('recover_timeout', 3.0)
        # autostart=true  →  begin in FOLLOW_LANE as soon as node starts.
        # Set false if you want to send 'START' manually first.
        self.declare_parameter('autostart', True)

        # ── state ────────────────────────────────────────────────────────────
        self._state                = 'STOP'
        self._lane_state           = 'LOST_BOTH'
        self._lane_state_received  = False   # stay in FOLLOW_LANE until first message arrives
        self._recover_start        = None
        self._recover_to           = self.get_parameter('recover_timeout').value

        # ── pub / sub ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, '/behavior/state', 10)

        self.create_subscription(String, '/lane/state',        self._on_lane,    10)
        self.create_subscription(String, '/behavior/command',  self._on_command, 10)

        # ── future perception inputs (wired but no-op until implemented) ─────
        # self.create_subscription(…, '/sign/detections',    self._on_sign,    10)
        # self.create_subscription(…, '/intersection/type',  self._on_inter,   10)

        self.create_timer(0.05, self._update)   # 20 Hz state machine tick

        if self.get_parameter('autostart').value:
            self._transition('FOLLOW_LANE')
        else:
            self._publish()

        self.get_logger().info(f'BehaviorManagerNode started — state: {self._state}')

    # ── transitions ──────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        if new_state not in self._ALL_STATES:
            self.get_logger().warn(f'Unknown state requested: {new_state}')
            return
        if new_state == self._state:
            return
        self.get_logger().info(f'{self._state} → {new_state}')
        self._state = new_state
        if new_state == 'RECOVER':
            self._recover_start = self.get_clock().now()
        self._publish()

    def _publish(self):
        msg      = String()
        msg.data = self._state
        self._pub.publish(msg)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_lane(self, msg: String):
        self._lane_state          = msg.data
        self._lane_state_received = True

    def _on_command(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd in self._CMD_MAP:
            self._transition(self._CMD_MAP[cmd])
        else:
            self.get_logger().warn(f'Unknown command: {cmd!r}')

    # ── state machine tick ───────────────────────────────────────────────────

    def _update(self):
        if self._state == 'FOLLOW_LANE':
            # Only react to lane loss after the detection node has sent at least one message.
            # Without this guard, the default LOST_BOTH initial value triggers RECOVER
            # immediately on startup before the camera pipeline is running.
            if self._lane_state_received and self._lane_state == 'LOST_BOTH':
                self._transition('RECOVER')

        elif self._state == 'RECOVER':
            if self._lane_state != 'LOST_BOTH':
                self._transition('FOLLOW_LANE')
            elif self._recover_start is not None:
                elapsed = (self.get_clock().now() - self._recover_start).nanoseconds * 1e-9
                if elapsed > self._recover_to:
                    self.get_logger().warn('Recovery timeout — full stop')
                    self._transition('STOP')

        # Future: APPROACH_INTERSECTION → read sign → TURN_* or GO_STRAIGHT
        # Future: WAIT_CROSSWALK        → timer or crosswalk clear → FOLLOW_LANE

        # Always re-publish so the controller never starves on a slow DDS QoS
        self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
