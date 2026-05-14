# robot_sim — Claude Context

## Project Goal
Build a ROS2 + Gazebo simulation environment to train an autonomous mobile robot (LIMO AGILEX type) using computer vision. The project progresses through checkpoints.

## System Setup
- OS: Ubuntu 22.04 (Jammy)
- ROS2: Humble Hawksbill — installed at `/opt/ros/humble`
- Gazebo: Classic 11 (binary at `gazebo`, sourced from system)
- Ignition Gazebo 6 (Fortress) also installed but NOT used here
- Shell: always source `/opt/ros/humble/setup.bash` AND `install/setup.bash` before running ROS2 commands

## Workspace Layout
```
robot_sim/                     ← colcon workspace root (also git root)
  src/
    robot_sim/                 ← ROS2 ament_cmake package
      package.xml
      CMakeLists.txt
      launch/
        sim.launch.py          ← basic world (drive_mode arg: diff|ackermann)
        track.launch.py        ← competition track world (same drive_mode arg)
      worlds/
        basic_world.world      ← flat ground + two traffic cones
        track_world.world      ← generated competition track (~39 <include> pieces)
      scripts/
        generate_track.py      ← regenerates track_world.world; run with python3
      models/traffic_cone/     ← custom Gazebo model (no system cone exists)
      models/track_straight_1m/     ← reusable 1 m straight track piece
      models/track_straight_0_5m/   ← reusable 0.5 m straight track piece
      models/track_arc_90/          ← reusable 90° arc, R=1.5 m
      models/track_t_intersection/  ← reusable T-junction (stem vertical, arm to +X)
      models/track_cross_intersection/ ← reusable 4-way cross
      urdf/limo_sim.urdf.xacro ← LIMO-like robot, dual drive mode
      config/rviz2.rviz
    lane_nav/                  ← ROS2 ament_python package (CP5 lane following)
      lane_nav/
        lane_detection_node.py ← IPM + sliding windows + polynomial fit → /lane/*
        lane_controller_node.py← PID error → /cmd_vel (Ackermann mode)
        behavior_manager_node.py← state machine; command bus for YOLO / intersections
      config/params.yaml       ← all tunable parameters with tuning notes
      launch/lane_nav.launch.py← starts all three nodes
  build/   ← gitignored
  install/ ← gitignored
  log/     ← gitignored
```

## Build & Run
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_sim sim.launch.py                        # basic world, diff-drive
ros2 launch robot_sim sim.launch.py drive_mode:=ackermann  # basic world, Ackermann
ros2 launch robot_sim track.launch.py                      # track world, diff-drive
ros2 launch robot_sim track.launch.py drive_mode:=ackermann # track world, Ackermann
```
Teleop (separate terminal):
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Autonomous lane following (Ackermann, after Gazebo is running):
```bash
# Terminal 1 — simulation
ros2 launch robot_sim track.launch.py drive_mode:=ackermann

# Terminal 2 — lane nav stack
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch lane_nav lane_nav.launch.py

# Terminal 3 — debug bird's-eye view
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run rqt_image_view rqt_image_view /lane/debug_image

# Manual control commands
ros2 topic pub /behavior/command std_msgs/String "data: STOP"
ros2 topic pub /behavior/command std_msgs/String "data: START"
ros2 topic echo /lane/lateral_error
```

To regenerate the track world after editing `generate_track.py`:
```bash
python3 src/robot_sim/scripts/generate_track.py
colcon build --symlink-install
```

## Robot: limo_sim (CP2 state)
- Visual: LIMO Pro look — white lower deck, black upper hood, tilted front cowl, green LED strips, dual antennas
- Chassis: 0.32×0.22×0.06m, white, 4 kg body mass
- Wheels: radius=0.05m, width=0.045m, track=0.17m, wheelbase=0.20m
- TF tree: `base_footprint` → `base_link` → wheels/steer links → camera_link → camera_optical_link
- **Topics (both modes):** `/cmd_vel` (in), `/odom` (out), `/joint_states` (out)
- **Camera topics:** `/camera/image_raw` (640×480 RGB @ 30 FPS), `/camera/camera_info`

### Diff mode (`drive_mode:=diff`)
- Plugin: `libgazebo_ros_diff_drive.so`, `num_wheel_pairs=2` (4WD skid-steer)
- All 4 wheels are fixed-axis continuous joints parented to `base_link`
- Node: `/limo_diff_drive`

### Ackermann mode (`drive_mode:=ackermann`)
- Plugin: `libgazebo_ros_ackermann_drive.so`
- Front 2 wheels: steered (steer revolute joint → wheel continuous joint, 2-DOF)
- Rear 2 wheels: driven (fixed-axis continuous joints)
- Steer limits: ±0.6 rad (≈34°); inner/outer wheels use correct Ackermann geometry
- Node: `/limo_ackermann_drive`
- Warning `steering_wheel_joint not found` is cosmetic (Prius demo artifact), not an error

### Camera sensor
- `libgazebo_ros_camera.so` on `camera_link`, 30 FPS, 640×480, 70° HFOV
- Optical frame: `camera_optical_link` (REP-103: x-right, y-down, z-forward)
- Mount: `xyz="0.135 0 0.10"` relative to `base_link` → **height ≈ 0.175 m above ground**, horizontal (no downward tilt)
- Derived VFOV ≈ 55.6°; focal length ≈ 457 px; horizon appears near y=300 in 640×480 image

## World: basic_world
- Flat ground plane + sun
- Two orange traffic cones (custom SDF — cylinder+cylinder, no mesh)
  - `traffic_cone_1` at (2.0, 0.5, 0)
  - `traffic_cone_2` at (2.0, -0.5, 0)
- Robot spawns at (0, 0, 0.05)
- `GAZEBO_MODEL_PATH` extended in launch file so Gazebo finds `model://traffic_cone`

## World: track_world
- Competition-style driving course matching a RoboCup-style circuit layout
- Generated by `scripts/generate_track.py` (edit and re-run to modify the track)
- **Track specs:** width=0.50 m, corner radius=1.50 m (centreline)
- **Markings:** white, visual-only (no collision — robot drives on ground plane)
  - Two continuous sidelines: 3 cm wide
  - Dashed centre line: 3 cm wide, 15 cm on / 8 cm gap
- **Road surface:** dark grey (#181818), 2 mm thin, visual-only
- **89 `<include>` elements** (37 inner-track pieces + 40 speed-road pieces + 10 signs + ground + sun)
- Robot spawns at (−4.0, −0.30, 0.05), yaw=π/2 (facing north, on outer-left road middle straight, between the two T-junctions)

### Two-level track: inner FIRA H-pattern + outer speed-road perimeter
The world contains two disjoint loops:
- **Inner track** (FIRA-style, with intersections and signs) — for sign-aware navigation
- **Speed road** (smooth perimeter, no intersections, no signs) — for unrestricted speed driving

#### Inner track (X=east, Y=north, all centreline coords)
```
              top T (0,3) ⊤
         ┌────────────────────┐
        /                      \
       │     ┌── upper ──┐      │
   ⊢ left T (-4,1)─cross(0,1)─right T (4,1) ⊣
       │       │       │        │
       │       │ inner │        │
       │       │  vert │        │
   ⊢ left T (-4,-1)─cross(0,-1)─right T (4,-1) ⊣
       │     └── lower ──┘      │
        \                      /
         └────────────────────┘
              bottom T (0,-3) ⊥
```
Outer loop: top y=3.0, bottom y=-3.0, left x=-4.0, right x=4.0; corners R=1.5 at (±2.5, ±1.5).
Inner roads:
- Upper horizontal: `y=1.0`, `x ∈ [−3.5, 3.5]` — joins left/right outer T's and crosses inner vertical
- Lower horizontal: `y=-1.0`, `x ∈ [−3.5, 3.5]` — same structure
- Inner vertical:   `x=0`,   `y ∈ [−2.5, 2.5]` — joins top/bottom outer T's, passes through both crosses

#### Speed road perimeter
Smooth oval surrounding the inner track with ~0.5 m clearance.
- Centerline rectangle: top y=5.5, bottom y=-5.5, left x=-6.5, right x=6.5
- Corner arcs R=1.5 at (±5.0, ±4.0)
- 40 pieces total: 4 arcs + 10×1m top + 10×1m bottom + 8×1m left + 8×1m right
- No T-intersections, no crosses, no signs — just a closed loop for speed practice

#### Signs (inner track only — 10 total)
Convention: directional sign placed BEFORE stop sign so robot reads "what to do" first, then "stop". 4 STOP+direction pairs at intersection approaches plus 2 standalone labels (NO-ENTRY on outer-right south of upper-right T arm; DEAD-END on lower-east inner road before lower-right T).

### Reusable track piece geometry
All pieces are visual-only (no collision), road surface at z=0.001, markings at z=0.004.
- **`track_straight_1m`**: 1 m along local X, centred at origin. Connect ends at x=±0.5.
- **`track_straight_0_5m`**: 0.5 m along local X, centred at origin. Connect ends at x=±0.25.
- **`track_arc_90`**: arc-centre at local origin, R=1.5 m, spans 0°→90° (first quadrant).
  Entry at local (1.5, 0) — road heading +Y. Exit at local (0, 1.5) — road heading −X.
  Rotate 0/90°/180°/270° to place TR/TL/BL/BR corners at world (±2.5, ±1.5).
- **`track_t_intersection`**: stem = 1 m vertical (x∈[−0.25,0.25], y∈[−0.5,0.5]);
  arm = 0.5 m × 0.5 m patch extending to +X (x∈[0,0.5], y∈[−0.25,0.25]).
  Connections: stem ends at (0, ±0.5), arm tip at (+0.5, 0).
  Rotate to orient: `yaw=0` arm→+X, `yaw=π/2` arm→+Y, `yaw=π` arm→−X, `yaw=3π/2` arm→−Y.
- **`track_cross_intersection`**: 1×0.5 horiz + 0.5×1 vert centred at origin.
  Connections at (±0.5, 0) and (0, ±0.5).

### `generate_track.py` approach
Uses a single `place(name, uri, x, y, yaw)` helper that emits `<include>` blocks.
Three sections: `build_inner_track()` (37 pieces), `build_speed_road()` (40 pieces),
`build_signs()` (10 signs).
Inner: 4 arcs + 6 T's + 2 crosses + 25 straight 1 m. Speed road: 4 arcs + 36 straight 1 m.
Signs: 4 STOP + 2 LEFT + 1 RIGHT + 1 FORWARD + 1 NO-ENTRY + 1 DEAD-END.

## Package: lane_nav (CP2.8 — autonomous lane following)

### Architecture
Three nodes in a clean pipeline:
```
/camera/image_raw
       │
       ▼
lane_detection_node  →  /lane/lateral_error  (Float64, −1…+1)
                     →  /lane/curvature      (Float64, pixels⁻¹)
                     →  /lane/state          (String: NORMAL|LOST_LEFT|LOST_RIGHT|LOST_BOTH)
                     →  /lane/debug_image    (Image, bird's-eye overlay)
                                                      │
behavior_manager_node  ←  /lane/state                │
                       ←  /behavior/command  (ext.)   │
                       →  /behavior/state    ─────────┤
                                                      │
lane_controller_node   ←  /lane/lateral_error         │
                       ←  /behavior/state  ←──────────┘
                       →  /cmd_vel
```

### Lane detection pipeline
1. Grayscale + GaussianBlur(5×5)
2. CLAHE (adaptive histogram equalisation, 8×8 tiles) — makes markings stand out regardless of global scene brightness
3. ROI-masked Otsu threshold: compute Otsu only on the road trapezoid pixels (floor: `white_thresh=80`) then mask non-road pixels to zero
4. IPM `cv2.warpPerspective` maps the road trapezoid to a bird's-eye rectangle
5. Histogram of bottom quarter → seed positions for the two line trackers
6. 9 sliding windows per line → pixel clouds for center dashed line and right sideline
7. `np.polyfit(y, x, 2)` → x = a·y² + b·y + c per line
8. Evaluate both polynomials at `lookahead_fraction` point (default 30 % from bottom of bird's-eye) → right lane centre x
9. `lateral_error = (right_lane_cx − w/2) / (lane_half_width_px)` — zero when centred in right lane
10. Exponential smoothing (α=0.75) and publish; `/lane/curvature` = lf[0] + rf[0] (sum of 'a' coefficients, px⁻¹)

### IPM source trapezoid defaults (640×480 image)
```
ipm_src: [80, 470, 560, 470, 420, 290, 220, 290]
          └─ near road ──┘    └─ far road (horizon) ─┘
```
Derived from camera geometry: height 0.175 m, no tilt, 70° HFOV.
Tune by watching `/lane/debug_image`: lane lines should be approximately vertical after the warp.

### PID + curvature feedforward sign convention
```
error > 0  →  robot left  of right-lane centre  →  steer RIGHT  (angular.z < 0)
error < 0  →  robot right of right-lane centre  →  steer LEFT   (angular.z > 0)
angular.z  =  −(Kp·e + Ki·∫e + Kd·ė)  −  Kff·curvature
```
Current gains: Kp=1.2, Ki=0.0, Kd=0.08, Kff_curvature=20.0.
Speed adapts: `v = base_speed × (1 − 0.5·|steer|/max_steer)`.

**Curvature feedforward rationale**: proportional-only control requires a persistent lateral error to maintain steering on a curve. At Kp=1.2 and a 1.5 m radius curve this steady-state offset is ~20 mm — acceptable on straights but visible on tight curves. The feedforward term uses `/lane/curvature` (sum of the 'a' polynomial coefficients, px⁻¹; positive = right curve) to inject the geometric steering signal before any error builds up. Kff=20 was derived from: steer_needed = atan(wheelbase/R) / curvature_px ≈ 32; tuned conservatively to 20 to avoid overcorrecting.

### Behavior state machine
Current states: `FOLLOW_LANE` → `RECOVER` (lane lost, 3 s timeout) → `STOP`
Future stub states already declared: `APPROACH_INTERSECTION`, `TURN_LEFT`, `TURN_RIGHT`, `GO_STRAIGHT`, `WAIT_CROSSWALK`
External command bus: publish `std_msgs/String` to `/behavior/command` (`START`, `STOP`, `TURN_LEFT`, …)

### Scalability hooks for CP5
- `/behavior/command` is the integration point for a YOLO sign-detector node — no changes to CV pipeline needed
- `behavior_manager_node._CMD_MAP` and `_ALL_STATES` are the only places to add new intersection logic
- `publish_debug: false` in params.yaml disables the debug image stream for deployment

## Key Technical Decisions

### Always
- **Gazebo Classic 11, not Ignition**: better LIMO package compatibility long-term
- **`ParameterValue(..., value_type=str)` required** around `Command(['xacro', ...])` in launch files — ROS2 Humble parses Command output as YAML otherwise and crashes
- **`--symlink-install`** means editing source files in `src/` takes effect without rebuilding
- **Kill stale processes before each launch**: `pkill -9 gzserver gzclient robot_state_publisher` — a leftover gzserver holds the lock and causes the new one to exit 255

### Ackermann stability — hard-won lessons
- `libgazebo_ros_ackermann_drive.so` uses force-based PID (NOT ODE velocity motors like diff_drive)
- **Root cause of NaN crashes**: plugin update rate was 100 Hz (10 ms); for wheel inertia 2.5e-4 kg·m², the closed-loop time constant τ_cl = I/(D_mech+P) ≈ 5.5 ms is below the Nyquist limit (T/2 = 5 ms) → sampled controller is unstable → oscillation → NaN
- **Fix 1: update_rate=500** (2 ms step; τ_cl/T = 2.75 → stable)
- **Fix 2: pause/unpause** after spawn (RegisterEventHandler in launch file) — prevents NaN from GetVelocity(0) returning uninitialized value on first physics step
- **Fix 3: linear PID = P=0.10, I=0, D=0**.  Earlier `I=0.5` wound up to 50% overshoot; the rear-left wheel reversed during left turns.
- **Fix 4: ackermann rear-wheel damping=0.02** (vs diff `0.005`).  The plugin closes its linear-velocity PID on `rear_right` alone but applies the same force to both rear wheels — during a left turn it commands negative force to slow the (faster) outer rear, which also pushes the (slower) inner-rear backwards.  Wheel-joint damping of 0.02 lets static friction (≤ 0.585 N·m) absorb the asymmetric force without the inner wheel reversing.  Steady-state forward speed = `P/(P+D)` of commanded ≈ 83 %.
- Steer PID uses P=2.0, D=0 — the steer joint has damping=0.3 Nm·s/rad which overdamps naturally
- **Fix 5: steer joint friction=0.0** (was 0.02 N·m Coulomb friction).  With friction=0.02 N·m, the joint would stick whenever the PID error was small (P×error < 0.02 N·m → joint doesn't move). This manifested as the front wheels locking at an intermediate steer angle during teleop diagonal keys and lane_nav curves; back-and-forth commands were needed to break the stiction. With friction=0.0, damping=0.3 provides the only resistance — no stiction, smooth position tracking.
- The Prius demo uses P=800 at 100 Hz because prius wheel inertia is ~2000× larger (0.586 vs 0.00025 kg·m²)

### Diff (4WD skid) tuning — hard-won lessons
- **mu2 must be LOW (0.15)** on all 4 wheels.  At `mu2=0.4` the lateral friction stick-slipped during turns: forces would build, suddenly release, then re-grip → visible chassis judder, jerky odom angular velocity (max|Δω|≈0.94 rad/s between samples).
- **`max_wheel_acceleration=6.0`** (was 1.0).  At 1.0 m/s² the velocity ramp took ~0.5 s to reach a 0.5 m/s target; teleop key taps barely moved the robot.
- **`max_wheel_torque=5`** (was 2). Pairs with the higher acceleration so the plugin can actually drive the wheels to target.
- Wheel-joint damping stays at 0.005 in diff mode.

## Checkpoints
- [x] **CP1** — Flat world, two cones, LIMO-like robot, teleop drive, RViz2
- [x] **CP2** — Dual drive modes (diff 4WD + Ackermann), active camera (30 FPS), RViz2
- [x] **CP2.5** — Competition track world (`track.launch.py`): outer loop + inner roads + lane markings, reusable model pieces
- [~] **CP2.8** — Autonomous lane following (`lane_nav` package): IPM + sliding window + PID + behavior state machine; right-lane tracking in Ackermann mode. Needs IPM calibration on first run; YOLO + intersection logic integration points are stubbed in.
- [ ] **CP3** — SLAM (slam_toolbox), build a map of the environment
- [ ] **CP4** — Nav2 autonomous navigation to goal poses
- [ ] **CP5** — Computer vision integration for cone/lane detection (YOLO sign detector, crosswalk/intersection handler — hooks ready in behavior_manager_node)

## Installed Packages (relevant)
`ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-plugins`, `ros-humble-xacro`,
`ros-humble-robot-state-publisher`, `ros-humble-rviz2`, `ros-humble-teleop-twist-keyboard`,
`ros-humble-diff-drive-controller`, `ros-humble-gz-ros2-control`,
`ros-humble-ackermann-steering-controller`, `ros-humble-controller-manager`,
`ros-humble-forward-command-controller`, `python3-colcon-common-extensions`

Not yet installed (needed for CP3+): `ros-humble-slam-toolbox`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`
