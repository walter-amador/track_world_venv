#!/usr/bin/env python3
"""
generate_track.py — Generate track_world.world from reusable track-piece models.

Run from anywhere:
    python3 generate_track.py

Outputs: src/robot_sim/worlds/track_world.world

Reusable models (src/robot_sim/models/):
  track_straight_1m        — 1 m straight, runs along local X, centred at origin
  track_straight_0_5m      — 0.5 m straight, runs along local X, centred at origin
  track_arc_90             — 90° arc, R=1.5 m, arc-centre at origin, spans 0°→90°
  track_t_intersection     — T-junction: 1 m N-S stem + 0.5 m arm to +X.
                             Stem footprint: x∈[-0.25,0.25], y∈[-0.5,0.5].
                             Arm footprint:  x∈[0,0.5],     y∈[-0.25,0.25].
                             Connections: (0, ±0.5) stem ends, (+0.5, 0) arm end.
  track_cross_intersection — 4-way cross, 1 m × 0.5 m horizontal + 0.5 m × 1 m
                             vertical, centred at origin.
                             Connections: (±0.5, 0) and (0, ±0.5).

Layout (X = east, Y = north):

  ── Inner FIRA H-pattern track (with 6 T's, 2 crosses, full sign set) ───────
  Outer loop: top y=3.0, bottom y=-3.0, left x=-4.0, right x=4.0
  Corners:    R=1.5 m arcs at (±2.5, ±1.5)
  Inner:      upper horiz y=1.0, lower horiz y=-1.0, vertical x=0
  6 T's at the 6 outer-loop entries, 2 crosses at (0, ±1.0)

  ── Speed road perimeter (no intersections, no signs) ──────────────────────
  Outer loop: top y=5.5, bottom y=-5.5, left x=-6.5, right x=6.5
  Corners:    R=1.5 m arcs at (±5.0, ±4.0)
  Surrounds the inner track with ~0.5 m clearance everywhere; provides a
  clean, sign-free oval for high-speed driving practice.
"""

import math
import os

# ── SDF include helper ────────────────────────────────────────────────────────

_INCLUDE_TMPL = """\
    <include>
      <name>{name}</name>
      <uri>model://{uri}</uri>
      <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.5f}</pose>
    </include>"""


def place(name, uri, x, y, z=0.0, yaw=0.0):
    return _INCLUDE_TMPL.format(name=name, uri=uri, x=x, y=y, z=z, yaw=yaw)


# ── Inner FIRA track ─────────────────────────────────────────────────────────

def build_inner_track():
    pieces = []

    # ── 4 corner arcs ─────────────────────────────────────────────────────────
    pieces.append(place("arc_tr", "track_arc_90",  2.5,  1.5, yaw=0))
    pieces.append(place("arc_tl", "track_arc_90", -2.5,  1.5, yaw=math.pi / 2))
    pieces.append(place("arc_bl", "track_arc_90", -2.5, -1.5, yaw=math.pi))
    pieces.append(place("arc_br", "track_arc_90",  2.5, -1.5, yaw=3 * math.pi / 2))

    # ── Outer top road  y=3.0 ────────────────────────────────────────────────
    # T at (0, 3.0) yaw=3π/2 has horizontal stem covering x ∈ [-0.5, 0.5].
    pieces.append(place("top_t", "track_t_intersection", 0.0, 3.0, yaw=3 * math.pi / 2))
    for i, x in enumerate([-2.0, -1.0, 1.0, 2.0]):
        pieces.append(place(f"top_{i + 1}", "track_straight_1m", x, 3.0, yaw=0))

    # ── Outer bottom road  y=-3.0 ────────────────────────────────────────────
    pieces.append(place("bot_t", "track_t_intersection", 0.0, -3.0, yaw=math.pi / 2))
    for i, x in enumerate([-2.0, -1.0, 1.0, 2.0]):
        pieces.append(place(f"bot_{i + 1}", "track_straight_1m", x, -3.0, yaw=0))

    # ── Outer left road  x=-4.0 ──────────────────────────────────────────────
    pieces.append(place("left_t_n", "track_t_intersection", -4.0,  1.0, yaw=0))
    pieces.append(place("left_mid", "track_straight_1m",    -4.0,  0.0, yaw=math.pi / 2))
    pieces.append(place("left_t_s", "track_t_intersection", -4.0, -1.0, yaw=0))

    # ── Outer right road  x=4.0 ──────────────────────────────────────────────
    pieces.append(place("right_t_n", "track_t_intersection",  4.0,  1.0, yaw=math.pi))
    pieces.append(place("right_mid", "track_straight_1m",     4.0,  0.0, yaw=math.pi / 2))
    pieces.append(place("right_t_s", "track_t_intersection",  4.0, -1.0, yaw=math.pi))

    # ── Cross intersections ──────────────────────────────────────────────────
    pieces.append(place("cross_n", "track_cross_intersection", 0.0,  1.0, yaw=0))
    pieces.append(place("cross_s", "track_cross_intersection", 0.0, -1.0, yaw=0))

    # ── Upper horizontal inner road  y=1.0 ───────────────────────────────────
    for i, x in enumerate([-3.0, -2.0, -1.0]):
        pieces.append(place(f"uh_w{i + 1}", "track_straight_1m", x, 1.0, yaw=0))
    for i, x in enumerate([1.0, 2.0, 3.0]):
        pieces.append(place(f"uh_e{i + 1}", "track_straight_1m", x, 1.0, yaw=0))

    # ── Lower horizontal inner road  y=-1.0 ──────────────────────────────────
    for i, x in enumerate([-3.0, -2.0, -1.0]):
        pieces.append(place(f"lh_w{i + 1}", "track_straight_1m", x, -1.0, yaw=0))
    for i, x in enumerate([1.0, 2.0, 3.0]):
        pieces.append(place(f"lh_e{i + 1}", "track_straight_1m", x, -1.0, yaw=0))

    # ── Inner vertical road  x=0 ─────────────────────────────────────────────
    pieces.append(place("iv_n", "track_straight_1m", 0.0,  2.0, yaw=math.pi / 2))
    pieces.append(place("iv_m", "track_straight_1m", 0.0,  0.0, yaw=math.pi / 2))
    pieces.append(place("iv_s", "track_straight_1m", 0.0, -2.0, yaw=math.pi / 2))

    return '\n'.join(pieces)


# ── Speed road perimeter ─────────────────────────────────────────────────────

def build_speed_road():
    """Outer rounded-rectangle perimeter for unrestricted speed driving.

    Centerline rectangle: y=±5.5, x=±6.5. Corner arcs R=1.5 at (±5.0, ±4.0).
    Top/bottom straights are 10 m each (10×1m), left/right are 8 m each (8×1m).
    Closest approach to the inner track is ~0.5 m at corner-to-corner points.
    """
    pieces = []

    # ── 4 corner arcs ─────────────────────────────────────────────────────────
    pieces.append(place("sr_arc_tr", "track_arc_90",  5.0,  4.0, yaw=0))
    pieces.append(place("sr_arc_tl", "track_arc_90", -5.0,  4.0, yaw=math.pi / 2))
    pieces.append(place("sr_arc_bl", "track_arc_90", -5.0, -4.0, yaw=math.pi))
    pieces.append(place("sr_arc_br", "track_arc_90",  5.0, -4.0, yaw=3 * math.pi / 2))

    # ── Top straight  y=5.5, x ∈ [-5.0, 5.0] ────────────────────────────────
    for i, x in enumerate([-4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5]):
        pieces.append(place(f"sr_top_{i + 1}", "track_straight_1m", x, 5.5, yaw=0))

    # ── Bottom straight  y=-5.5 ──────────────────────────────────────────────
    for i, x in enumerate([-4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5]):
        pieces.append(place(f"sr_bot_{i + 1}", "track_straight_1m", x, -5.5, yaw=0))

    # ── Left straight  x=-6.5, y ∈ [-4.0, 4.0] ──────────────────────────────
    for i, y in enumerate([-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]):
        pieces.append(place(f"sr_left_{i + 1}", "track_straight_1m", -6.5, y, yaw=math.pi / 2))

    # ── Right straight  x=6.5 ────────────────────────────────────────────────
    for i, y in enumerate([-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]):
        pieces.append(place(f"sr_right_{i + 1}", "track_straight_1m", 6.5, y, yaw=math.pi / 2))

    return '\n'.join(pieces)


# ── Traffic sign placements ───────────────────────────────────────────────────
#
# Convention:
#   Plate thin axis is local Y. To face approaching traffic:
#     +Y traffic (north): yaw=π     plate normal=-Y
#     -Y traffic (south): yaw=0     plate normal=+Y
#     +X traffic (east):  yaw=π/2   plate normal=-X
#     -X traffic (west):  yaw=-π/2  plate normal=+X
#   Right-side offset (centerline → sign):
#     +Y traffic: (+0.35, 0)        -Y traffic: (-0.35, 0)
#     +X traffic: (0, -0.35)        -X traffic: (0, +0.35)
#
# Logic at each intersection approach: place a directional sign FIRST
# (further from intersection so the robot reads it first while approaching),
# then a STOP sign closer to the intersection. Pair = "what to do" + "stop".
#
# Robot spawns at (-4, -0.30) heading +Y. It encounters Pair 1 first.
# Speed road is intentionally sign-free.
#
def build_signs():
    signs = []

    # ── Pair 1: Upper-left T at (-4, 1.0), approaching from south (+Y) ──────
    #   Direction sign first (RIGHT — turn right onto upper inner road),
    #   then STOP just before the T stem (T occupies y∈[0.5, 1.5]).
    signs.append(place("sign_right_1", "sign_right", -3.65, 0.10, yaw=math.pi))
    signs.append(place("sign_stop_1", "sign_stop",  -3.65, 0.45, yaw=math.pi))

    # ── Pair 2: Upper cross at (0, 1.0), approaching from west (+X) ─────────
    #   Direction sign first (LEFT — left turn option = north onto inner vertical),
    #   then STOP just before the cross (cross occupies x∈[-0.5, 0.5]).
    signs.append(place("sign_left_1", "sign_left", -1.10, 0.65, yaw=math.pi / 2))
    signs.append(place("sign_stop_2", "sign_stop", -0.60, 0.65, yaw=math.pi / 2))

    # ── Pair 3: Top T at (0, 3.0), approaching from south on inner vertical ─
    #   Direction sign first (LEFT — turn left onto outer top road heading west),
    #   then STOP just before the T arm tip (arm tip at y=2.5).
    signs.append(place("sign_left_2", "sign_left", 0.35, 2.10, yaw=math.pi))
    signs.append(place("sign_stop_3", "sign_stop", 0.35, 2.45, yaw=math.pi))

    # ── Pair 4: Lower cross at (0, -1.0), approaching from north (-Y) ───────
    #   Direction sign first (FORWARD — go straight through the cross),
    #   then STOP just before the cross north edge at y=-0.5.
    signs.append(place("sign_forward_1", "sign_forward", -0.35, -0.10, yaw=0))
    signs.append(place("sign_stop_4",    "sign_stop",    -0.35, -0.45, yaw=0))

    # ── Standalone labels ───────────────────────────────────────────────────
    # NO-ENTRY: outer-right road, southbound, just north of upper-right T arm
    # entry — labels the right-side T branch as no-entry from the outer loop.
    signs.append(place("sign_no_entry_1", "sign_no_entry", 3.65, 1.55, yaw=0))

    # DEAD-END: lower horizontal inner road, eastbound, before lower-right T —
    # labels the eastward branch as a dead-end route at the T.
    signs.append(place("sign_dead_end_1", "sign_dead_end", 3.20, -1.35, yaw=math.pi / 2))

    return '\n'.join(signs)


# ── World file template ───────────────────────────────────────────────────────

_WORLD_TEMPLATE = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="track_world">

    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type><iters>150</iters><sor>1.3</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm><erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>1.0 0.0 22.0 0 1.55 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

{inner}

{speed}

{signs}
  </world>
</sdf>
"""


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    inner_sdf = build_inner_track()
    speed_sdf = build_speed_road()
    signs_sdf = build_signs()
    world_content = _WORLD_TEMPLATE.format(
        inner=inner_sdf, speed=speed_sdf, signs=signs_sdf
    )

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.normpath(
        os.path.join(script_dir, '..', 'worlds', 'track_world.world')
    )

    with open(out_path, 'w') as fh:
        fh.write(world_content)

    n_includes = world_content.count('<include>')
    print(f"Written  : {out_path}")
    print(f"<include> elements: {n_includes}")
