#!/usr/bin/env python3
"""
generate_track.py — Generate track_world.world for robot_sim.

Run from anywhere:
    python3 generate_track.py

Outputs: src/robot_sim/worlds/track_world.world

Track specs:
  - Track width:       0.50 m
  - Corner radius:     1.50 m (centerline)
  - Continuous side lines:  white, 3 cm wide
  - Dashed centre line:     white, 3 cm wide, 15 cm dash / 8 cm gap
  - Road surface:      dark grey, visual-only (robot drives on ground plane)
  - Markings:          visual-only, 2 mm raised above road surface

Layout (X = east, Y = north, top view):
  ┌─────────────────────────────────────────────┐
  │  Large outer rounded-rectangle loop         │
  │  ┌──────────────────────────────────┐       │
  │  │  Inner cross roads + sub-roads   │       │
  │  └──────────────────────────────────┘       │
  │                               ══════════════╡ exit road (right)
  └─────────────────────────────────────────────┘

Key coordinates (road centrelines):
  Outer top:      y =  3.0,  x ∈ [−2.5, 2.5]
  Outer bottom:   y = −3.0,  x ∈ [−2.5, 2.5]
  Outer left:     x = −4.0,  y ∈ [−1.5, 1.5]
  Outer right:    x =  4.0,  y ∈ [−1.5, 1.5]
  Corners:        R = 1.5 m, centres at (±2.5, ±1.5)
  Exit road:      y =  0.0,  x ∈ [4.25, 6.25]
  Inner horiz:    y =  0.0,  x ∈ [−3.75, 3.75]
  Inner vert:     x =  0.0,  y ∈ [−2.75, 2.75]
  Inner UL:       y =  1.25, x ∈ [−3.75, −0.25]
  Inner LR:       y = −1.25, x ∈ [ 0.25, 3.75]
"""

import math
import os

# ── Track geometry parameters ─────────────────────────────────────────────────
TW       = 0.50    # track width (m)
R        = 1.50    # corner arc radius — centreline (m)

# Road surface (visual only — robot drives on ground plane)
RZ       = 0.001   # box centre Z  → surface sits 0–2 mm above ground
RD       = 0.002   # box depth (2 mm)

# Lane markings (on top of road surface)
MZ       = 0.004   # box centre Z  → marking sits 3–5 mm above ground
MD       = 0.002   # marking depth (2 mm)
MW       = 0.030   # marking width (3 cm)
DL       = 0.150   # dash length (15 cm)
GL       = 0.080   # gap between dashes (8 cm)

SIDE_OFF = TW / 2 - MW / 2   # 0.235 m — offset from centreline to sideline centre

# Colours  (r, g, b)
ROAD_CLR = (0.18, 0.18, 0.18)   # dark asphalt
MARK_CLR = (1.00, 1.00, 1.00)   # white

# Arc resolution
DEG_PER_SEG = 8   # degrees of arc per box segment


# ── SDF helpers ───────────────────────────────────────────────────────────────

_idx = [0]

def _uid():
    _idx[0] += 1
    return f"t{_idx[0]:05d}"


def _box(cx, cy, cz, lx, ly, lz, yaw, color):
    """Return SDF for a static, visual-only box model."""
    r, g, b = color
    name = _uid()
    return (
        f'\n    <model name="{name}">'
        f'<static>true</static>'
        f'<pose>{cx:.5f} {cy:.5f} {cz:.5f} 0 0 {yaw:.5f}</pose>'
        f'<link name="L">'
        f'<visual name="v">'
        f'<geometry><box><size>{lx:.5f} {ly:.5f} {lz:.5f}</size></box></geometry>'
        f'<material>'
        f'<ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>'
        f'<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
        f'<specular>0.05 0.05 0.05 1</specular>'
        f'</material>'
        f'</visual>'
        f'</link>'
        f'</model>'
    )


# ── Straight segment ──────────────────────────────────────────────────────────

def straight(cx, cy, length, yaw=0.0):
    """
    Generate road surface + sidelines + dashed centreline
    for a straight road section centred at (cx, cy).

    yaw = 0      → road runs along world X axis
    yaw = π/2    → road runs along world Y axis
    """
    parts = []

    # ── Road surface
    parts.append(_box(cx, cy, RZ, length, TW, RD, yaw, ROAD_CLR))

    # ── Sidelines (parallel to road, offset perpendicular)
    px = -math.sin(yaw)   # perpendicular unit-vector X component
    py =  math.cos(yaw)   # perpendicular unit-vector Y component
    for sign in (+1, -1):
        sx = cx + sign * SIDE_OFF * px
        sy = cy + sign * SIDE_OFF * py
        parts.append(_box(sx, sy, MZ, length, MW, MD, yaw, MARK_CLR))

    # ── Dashed centreline
    ax = math.cos(yaw)    # along-road unit-vector X component
    ay = math.sin(yaw)    # along-road unit-vector Y component
    step  = DL + GL
    n     = max(1, int(length / step))
    start = -(n * DL + (n - 1) * GL) / 2 + DL / 2
    for i in range(n):
        off = start + i * step
        parts.append(_box(cx + off * ax, cy + off * ay, MZ, DL, MW, MD, yaw, MARK_CLR))

    return ''.join(parts)


# ── Arc segment ───────────────────────────────────────────────────────────────

def _arc_strip(acx, acy, r, a0, a1, width, z, depth, color):
    """
    Place overlapping boxes along an arc at radius `r` from (acx, acy).
    a0, a1 in radians; direction: increasing angle (CCW in standard math).
    """
    parts = []
    n   = max(4, int(abs(math.degrees(a1 - a0)) / DEG_PER_SEG))
    dth = (a1 - a0) / n
    for i in range(n):
        th   = a0 + (i + 0.5) * dth
        bx   = acx + r * math.cos(th)
        by   = acy + r * math.sin(th)
        tang = th + math.pi / 2           # tangent = 90° from radius
        slen = r * abs(dth) * 1.08        # arc-length with small overlap
        parts.append(_box(bx, by, z, slen, width, depth, tang, color))
    return ''.join(parts)


def arc(acx, acy, start_deg, end_deg):
    """
    Generate road surface + sidelines + dashed centreline for one arc.
    Angles in degrees, increasing CCW (standard math convention).

    Corner examples (road centreline at R = 1.5 m):
      TL: arc(-2.5,  1.5,  90, 180)
      TR: arc( 2.5,  1.5,   0,  90)
      BL: arc(-2.5, -1.5, 180, 270)
      BR: arc( 2.5, -1.5, 270, 360)
    """
    parts = []
    a0 = math.radians(start_deg)
    a1 = math.radians(end_deg)

    # Road surface at centreline radius
    parts.append(_arc_strip(acx, acy, R,            a0, a1, TW, RZ, RD, ROAD_CLR))

    # Sidelines at offset radii
    for r_off in (+SIDE_OFF, -SIDE_OFF):
        parts.append(_arc_strip(acx, acy, R + r_off, a0, a1, MW, MZ, MD, MARK_CLR))

    # Dashed centreline along the arc
    span    = abs(a1 - a0)
    arc_len = R * span
    step    = DL + GL
    n_dash  = max(1, int(arc_len / step))
    total   = n_dash * DL + (n_dash - 1) * GL
    # centre the dash pattern within the arc
    pad     = (arc_len - total) / 2
    dir_    = 1 if a1 > a0 else -1
    for i in range(n_dash):
        a_d  = a0 + dir_ * (pad + i * step + DL / 2) / R
        bx   = acx + R * math.cos(a_d)
        by   = acy + R * math.sin(a_d)
        tang = a_d + math.pi / 2
        parts.append(_box(bx, by, MZ, DL, MW, MD, tang, MARK_CLR))

    return ''.join(parts)


# ── Full track geometry ───────────────────────────────────────────────────────

def build_track():
    segs = []

    # ════ OUTER LOOP ═════════════════════════════════════════════════════════
    # Straight sections (centreline coords)
    segs.append(straight( 0.0,  3.0, 5.0, yaw=0.0))           # top
    segs.append(straight(-4.0,  0.0, 3.0, yaw=math.pi/2))     # left
    segs.append(straight( 4.0,  0.0, 3.0, yaw=math.pi/2))     # right
    segs.append(straight( 0.0, -3.0, 5.0, yaw=0.0))           # bottom

    # 90° corner arcs (arc-centre, start°, end°)
    segs.append(arc(-2.5,  1.5,  90, 180))   # top-left
    segs.append(arc( 2.5,  1.5,   0,  90))   # top-right
    segs.append(arc(-2.5, -1.5, 180, 270))   # bottom-left
    segs.append(arc( 2.5, -1.5, 270, 360))   # bottom-right

    # ════ RIGHT EXIT ROAD ════════════════════════════════════════════════════
    # Centre at (5.25, 0.0), from x≈4.25 to x≈6.25 (2.0 m), T-joined to right outer
    segs.append(straight(5.25, 0.0, 2.0, yaw=0.0))

    # ════ INNER ROAD NETWORK ═════════════════════════════════════════════════
    # Inner horizontal:  y = 0.0,   x ∈ [−3.75, 3.75]  (7.5 m)
    segs.append(straight(0.0,  0.0, 7.5, yaw=0.0))
    # Inner vertical:    x = 0.0,   y ∈ [−2.75, 2.75]  (5.5 m)
    segs.append(straight(0.0,  0.0, 5.5, yaw=math.pi/2))
    # Upper-left inner:  y = 1.25,  x ∈ [−3.75, −0.25]  (3.5 m) — asymmetric sub-loop
    segs.append(straight(-2.0, 1.25, 3.5, yaw=0.0))
    # Lower-right inner: y = −1.25, x ∈ [0.25, 3.75]  (3.5 m) — asymmetric sub-loop
    segs.append(straight( 2.0,-1.25, 3.5, yaw=0.0))

    return ''.join(segs)


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
        <!-- Overhead view centred on the track -->
        <pose>1.0 0.0 20.0 0 1.55 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
{track}
  </world>
</sdf>
"""


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    track_sdf = build_track()
    world_content = _WORLD_TEMPLATE.format(track=track_sdf)

    # Write next to the worlds/ directory relative to this script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_path   = os.path.normpath(
        os.path.join(script_dir, '..', 'worlds', 'track_world.world')
    )

    with open(out_path, 'w') as fh:
        fh.write(world_content)

    n_models = world_content.count('<model name=')
    print(f"Written  : {out_path}")
    print(f"SDF models generated: {n_models}")
