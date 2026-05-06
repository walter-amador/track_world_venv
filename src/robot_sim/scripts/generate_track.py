#!/usr/bin/env python3
"""
generate_track.py — Generate track_world.world from reusable track-piece models.

Run from anywhere:
    python3 generate_track.py

Outputs: src/robot_sim/worlds/track_world.world

Reusable models (src/robot_sim/models/):
  track_straight_1m      — 1 m straight, runs along local X, centred at origin
  track_straight_0_5m    — 0.5 m straight, runs along local X, centred at origin
  track_arc_90           — 90° arc, R=1.5 m, arc-centre at origin, spans 0°→90°
  track_t_intersection   — T-junction: vertical stem + 1 m arm extending to +X
  track_cross_intersection — 4-way cross, ±0.75 m in X and Y from origin

Layout (X = east, Y = north):
  Outer loop: top y=3.0, bottom y=-3.0, left x=-4.0, right x=4.0
  Corners:    R=1.5 m arcs at (±2.5, ±1.5)
  Exit road:  y=0, x ∈ [4.25, 6.25]  (T arm + 1 extra 1 m piece)
  Inner cross:  centre at (0, 0)
  Inner horiz:  y=0,    x ∈ [-3.75, 3.75]
  Inner vert:   x=0,    y ∈ [-2.75, 2.75]
  Upper-left:   y=1.25, x ∈ [-3.75, -0.25]
  Lower-right:  y=-1.25,x ∈ [ 0.25,  3.75]
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
    # track_arc_90: arc-centre at model origin; entry at local (1.5, 0) heading +Y,
    # exit at local (0, 1.5) heading -X.  Rotate by 0/90/180/270° for each corner.
    pieces.append(place("arc_tr", "track_arc_90",  2.5,  1.5, yaw=0))
    pieces.append(place("arc_tl", "track_arc_90", -2.5,  1.5, yaw=math.pi / 2))
    pieces.append(place("arc_bl", "track_arc_90", -2.5, -1.5, yaw=math.pi))
    pieces.append(place("arc_br", "track_arc_90",  2.5, -1.5, yaw=3 * math.pi / 2))

    # ── Top outer road  y=3.0, x ∈ [-2.5, 2.5] ───────────────────────────────
    for i, x in enumerate([-2.0, -1.0, 0.0, 1.0, 2.0]):
        pieces.append(place(f"top_{i + 1}", "track_straight_1m", x, 3.0, yaw=0))

    # ── Bottom outer road  y=-3.0, x ∈ [-2.5, 2.5] ───────────────────────────
    for i, x in enumerate([-2.0, -1.0, 0.0, 1.0, 2.0]):
        pieces.append(place(f"bot_{i + 1}", "track_straight_1m", x, -3.0, yaw=0))

    # ── Left outer road  x=-4.0, y ∈ [-1.5, 1.5] ────────────────────────────
    # T stem covers y ∈ [-0.5, 0.5]; T arm extends right as inner-horizontal start.
    pieces.append(place("left_n", "track_straight_1m",    -4.0,  1.0, yaw=math.pi / 2))
    pieces.append(place("left_t", "track_t_intersection", -4.0,  0.0, yaw=0))
    pieces.append(place("left_s", "track_straight_1m",    -4.0, -1.0, yaw=math.pi / 2))

    # ── Right outer road  x=4.0, y ∈ [-1.5, 1.5] + exit road to x=6.25 ──────
    # T stem covers y ∈ [-0.5, 0.5]; T arm covers x ∈ [4.25, 5.25] (exit road).
    pieces.append(place("right_n", "track_straight_1m",     4.0,  1.0, yaw=math.pi / 2))
    pieces.append(place("right_t", "track_t_intersection",  4.0,  0.0, yaw=0))
    pieces.append(place("right_s", "track_straight_1m",     4.0, -1.0, yaw=math.pi / 2))
    pieces.append(place("exit_1",  "track_straight_1m",     5.75, 0.0, yaw=0))

    # ── Center cross intersection ──────────────────────────────────────────────
    pieces.append(place("cross", "track_cross_intersection", 0.0, 0.0, yaw=0))

    # ── Inner horizontal  y=0, x ∈ [-3.75, 3.75] ─────────────────────────────
    # West of cross: left T arm covers x ∈ [-3.75, -2.75]; 2 more pieces follow.
    for i, x in enumerate([-2.25, -1.25]):
        pieces.append(place(f"ih_w{i + 1}", "track_straight_1m", x, 0.0, yaw=0))
    # East of cross: 3 pieces reach x=3.75 (right T stem bridges to outer road).
    for i, x in enumerate([1.25, 2.25, 3.25]):
        pieces.append(place(f"ih_e{i + 1}", "track_straight_1m", x, 0.0, yaw=0))

    # ── Inner vertical  x=0, y ∈ [-2.75, 2.75] ───────────────────────────────
    # Outer top/bottom road edges are at y=±2.75 — these pieces abut them exactly.
    for i, y in enumerate([-2.25, -1.25]):
        pieces.append(place(f"iv_s{i + 1}", "track_straight_1m", 0.0, y, yaw=math.pi / 2))
    for i, y in enumerate([1.25, 2.25]):
        pieces.append(place(f"iv_n{i + 1}", "track_straight_1m", 0.0, y, yaw=math.pi / 2))

    # ── Upper-left inner road  y=1.25, x ∈ [-3.75, -0.25] ───────────────────
    # 3 × 1 m + 1 × 0.5 m = 3.5 m total.
    for i, x in enumerate([-3.25, -2.25, -1.25]):
        pieces.append(place(f"ul_{i + 1}", "track_straight_1m",   x, 1.25, yaw=0))
    pieces.append(place("ul_4", "track_straight_0_5m", -0.5, 1.25, yaw=0))

    # ── Lower-right inner road  y=-1.25, x ∈ [0.25, 3.75] ───────────────────
    # 1 × 0.5 m + 3 × 1 m = 3.5 m total.
    pieces.append(place("lr_1", "track_straight_0_5m",  0.5, -1.25, yaw=0))
    for i, x in enumerate([1.25, 2.25, 3.25]):
        pieces.append(place(f"lr_{i + 2}", "track_straight_1m", x, -1.25, yaw=0))

    return '\n'.join(pieces)


# ── Traffic sign placements ───────────────────────────────────────────────────
#
# Signs follow real road convention: on the RIGHT side of the road, face toward
# approaching traffic.  Yaw key (box thin axis = Y, face is the ±Y plane):
#   yaw=π   → face points south  (for northbound roads, robot approaching from S)
#   yaw=π/2 → face points west   (for eastbound roads,  robot approaching from W)
#
# Road width = 0.50 m; sign offset = 0.35 m from centreline (just outside edge).
#
# Layout reminder (centrelines):
#   Left outer: x=-4.0, N-S, T-junction at y=0 (arm east toward inner horiz)
#   Inner horiz: y=0,   E-W, cross at (0,0)
#   Inner vert:  x=0,   N-S, cross at (0,0), lower-right arm at y=-1.25
#   Exit road:   y=0, x ∈ [4.25, 6.25], eastbound
#
def build_signs():
    signs = []

    # Robot goes north along left outer road (x=-4) from spawn (-4,-0.75).
    # Upcoming T-junction at y=0 branches right onto inner-horizontal road.
    # FORWARD: "continue straight ahead (north)".
    signs.append(place("sign_forward_1", "sign_forward", -3.65, -0.60, yaw=math.pi))

    # Robot enters inner horizontal (y=0) going east after the left T-junction.
    # Cross intersection (0,0) is ahead — a left turn (north) is available.
    # LEFT: "left turn option at cross".
    signs.append(place("sign_left_1", "sign_left", -0.85, -0.35, yaw=math.pi / 2))

    # Robot going north on inner vertical (x=0); lower-right road branches east at y=-1.25.
    # RIGHT: "right turn option ahead".
    signs.append(place("sign_right_1", "sign_right", 0.35, -1.75, yaw=math.pi))

    # Same northbound inner vertical road, approaching cross intersection at y=0.
    # STOP: "stop before entering the cross".
    signs.append(place("sign_stop_1", "sign_stop", 0.35, -0.80, yaw=math.pi))

    # Exit road (y=0, x > 4.25) is one-way eastbound.
    # NO ENTRY: mid-road, reminds that re-entry going west is not allowed.
    signs.append(place("sign_no_entry_1", "sign_no_entry", 5.00, -0.35, yaw=math.pi / 2))

    # End of exit road at x ≈ 6.25.
    # DEAD END: road terminates ahead.
    signs.append(place("sign_dead_end_1", "sign_dead_end", 6.10, -0.35, yaw=math.pi / 2))

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
    print(f"<include> elements: {n_includes}  (was 371 <model> boxes)")
