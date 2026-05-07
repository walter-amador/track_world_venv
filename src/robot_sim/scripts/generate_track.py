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

FIRA-style layout (X = east, Y = north):
  Outer loop: top y=3.0, bottom y=-3.0, left x=-4.0, right x=4.0
  Corners:    R=1.5 m arcs at (±2.5, ±1.5)
  Inner roads form an H-pattern:
    - Upper horizontal at y=1.0, x ∈ [-3.5, 3.5]
    - Lower horizontal at y=-1.0, x ∈ [-3.5, 3.5]
    - Vertical at x=0,  y ∈ [-2.5, 2.5]
  6 T-intersections: 4 on outer left/right at y=±1.0, plus top (0,3) and bottom (0,-3).
  2 cross intersections at (0, ±1.0).
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


# ── Track layout ─────────────────────────────────────────────────────────────

def build_track():
    pieces = []

    # ── 4 corner arcs ─────────────────────────────────────────────────────────
    pieces.append(place("arc_tr", "track_arc_90",  2.5,  1.5, yaw=0))
    pieces.append(place("arc_tl", "track_arc_90", -2.5,  1.5, yaw=math.pi / 2))
    pieces.append(place("arc_bl", "track_arc_90", -2.5, -1.5, yaw=math.pi))
    pieces.append(place("arc_br", "track_arc_90",  2.5, -1.5, yaw=3 * math.pi / 2))

    # ── Outer top road  y=3.0 ────────────────────────────────────────────────
    # T at (0, 3.0) yaw=3π/2 has horizontal stem covering x ∈ [-0.5, 0.5].
    # Two 2 m gaps to corner exits at x=±2.5 → 2×1m on each side.
    pieces.append(place("top_t", "track_t_intersection", 0.0, 3.0, yaw=3 * math.pi / 2))
    for i, x in enumerate([-2.0, -1.0, 1.0, 2.0]):
        pieces.append(place(f"top_{i + 1}", "track_straight_1m", x, 3.0, yaw=0))

    # ── Outer bottom road  y=-3.0 ────────────────────────────────────────────
    # T at (0, -3.0) yaw=π/2 has horizontal stem covering x ∈ [-0.5, 0.5].
    pieces.append(place("bot_t", "track_t_intersection", 0.0, -3.0, yaw=math.pi / 2))
    for i, x in enumerate([-2.0, -1.0, 1.0, 2.0]):
        pieces.append(place(f"bot_{i + 1}", "track_straight_1m", x, -3.0, yaw=0))

    # ── Outer left road  x=-4.0 ──────────────────────────────────────────────
    # Two T's at y=±1.0 (yaw=0) cover y ∈ [-1.5, -0.5] and [0.5, 1.5].
    # Single 1 m straight bridges the middle (y ∈ [-0.5, 0.5]).
    pieces.append(place("left_t_n", "track_t_intersection", -4.0,  1.0, yaw=0))
    pieces.append(place("left_mid", "track_straight_1m",    -4.0,  0.0, yaw=math.pi / 2))
    pieces.append(place("left_t_s", "track_t_intersection", -4.0, -1.0, yaw=0))

    # ── Outer right road  x=4.0 ──────────────────────────────────────────────
    # Two T's at y=±1.0 (yaw=π) — arms extend to -X (inward).
    pieces.append(place("right_t_n", "track_t_intersection",  4.0,  1.0, yaw=math.pi))
    pieces.append(place("right_mid", "track_straight_1m",     4.0,  0.0, yaw=math.pi / 2))
    pieces.append(place("right_t_s", "track_t_intersection",  4.0, -1.0, yaw=math.pi))

    # ── Cross intersections ──────────────────────────────────────────────────
    pieces.append(place("cross_n", "track_cross_intersection", 0.0,  1.0, yaw=0))
    pieces.append(place("cross_s", "track_cross_intersection", 0.0, -1.0, yaw=0))

    # ── Upper horizontal inner road  y=1.0 ───────────────────────────────────
    # Left T arm tip at x=-3.5 → upper cross west connection at x=-0.5: 3 m → 3×1m
    # Upper cross east connection at x=0.5 → right T arm tip at x=3.5: 3 m → 3×1m
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
    # Top T arm tip at y=2.5 → upper cross north connection at y=1.5: 1 m → 1×1m
    # Upper cross south at y=0.5 → lower cross north at y=-0.5:        1 m → 1×1m
    # Lower cross south at y=-1.5 → bottom T arm tip at y=-2.5:        1 m → 1×1m
    pieces.append(place("iv_n", "track_straight_1m", 0.0,  2.0, yaw=math.pi / 2))
    pieces.append(place("iv_m", "track_straight_1m", 0.0,  0.0, yaw=math.pi / 2))
    pieces.append(place("iv_s", "track_straight_1m", 0.0, -2.0, yaw=math.pi / 2))

    return '\n'.join(pieces)


# ── Traffic sign placements ───────────────────────────────────────────────────
#
# Signs sit on the RIGHT side of the road, plate facing approaching traffic.
# Plate thin axis is local Y, so:
#   robot going +Y (north): yaw=π   (plate normal = world -Y)
#   robot going -Y (south): yaw=0   (plate normal = world +Y)
#   robot going +X (east):  yaw=π/2 (plate normal = world -X)
#   robot going -X (west):  yaw=-π/2
# Right-side offset for each direction (road centre → sign position):
#   +Y traffic:  +X side, offset (+0.35, 0)
#   -Y traffic:  -X side, offset (-0.35, 0)
#   +X traffic:  -Y side, offset (0, -0.35)
#   -X traffic:  +Y side, offset (0, +0.35)
#
# Robot spawns at (-4, -0.3) heading +Y (north). Signs are placed along its
# expected exploration path through the upper-left T → upper cross → top T,
# then around the lower half of the track.
#
def build_signs():
    signs = []

    # ── Outer-left road, northbound, approaching upper-left T at y=1.0 ──────
    # STOP just south of the T stem entry (T occupies y ∈ [0.5, 1.5]).
    signs.append(place("sign_stop_1", "sign_stop", -3.65, 0.55, yaw=math.pi))

    # ── On upper inner road (y=1.0), eastbound just past upper-left T arm ───
    # RIGHT — confirms the right-turn manoeuvre that just took place.
    signs.append(place("sign_right_1", "sign_right", -3.20, 0.65, yaw=math.pi / 2))

    # ── Eastbound on upper horizontal, approaching upper cross at (0, 1.0) ──
    # STOP before entering the cross.
    signs.append(place("sign_stop_2", "sign_stop", -0.85, 0.65, yaw=math.pi / 2))

    # ── Northbound on inner vertical, approaching upper cross from south ────
    # LEFT — west turn option onto upper horizontal.
    signs.append(place("sign_left_1", "sign_left", 0.35, 0.35, yaw=math.pi))

    # ── Northbound on inner vertical, approaching top T at (0, 3.0) ─────────
    # STOP at end of inner vertical before joining outer top road.
    signs.append(place("sign_stop_3", "sign_stop", 0.35, 2.45, yaw=math.pi))

    # ── Northbound on inner vertical between lower and upper cross ──────────
    # FORWARD — keep going straight, upper cross ahead has full 4-way options.
    signs.append(place("sign_forward_1", "sign_forward", 0.35, -0.40, yaw=math.pi))

    # ── Eastbound on lower horizontal, approaching lower cross from west ────
    # RIGHT — south turn option (toward bottom T).
    signs.append(place("sign_right_2", "sign_right", -0.85, -1.35, yaw=math.pi / 2))

    # ── Southbound on inner vertical, approaching bottom T at (0, -3.0) ─────
    # STOP at end of inner vertical before joining outer bottom road.
    signs.append(place("sign_stop_4", "sign_stop", -0.35, -2.45, yaw=0))

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
        <pose>1.0 0.0 20.0 0 1.55 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

{track}

{signs}
  </world>
</sdf>
"""


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    track_sdf = build_track()
    signs_sdf = build_signs()
    world_content = _WORLD_TEMPLATE.format(track=track_sdf, signs=signs_sdf)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.normpath(
        os.path.join(script_dir, '..', 'worlds', 'track_world.world')
    )

    with open(out_path, 'w') as fh:
        fh.write(world_content)

    n_includes = world_content.count('<include>')
    print(f"Written  : {out_path}")
    print(f"<include> elements: {n_includes}")
