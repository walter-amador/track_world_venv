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
      launch/sim.launch.py     ← main entry point (drive_mode arg: diff|ackermann)
      worlds/basic_world.world
      models/traffic_cone/     ← custom Gazebo model (no system cone exists)
      urdf/limo_sim.urdf.xacro ← LIMO-like robot, dual drive mode
      config/rviz2.rviz
  build/   ← gitignored
  install/ ← gitignored
  log/     ← gitignored
```

## Build & Run
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_sim sim.launch.py                        # diff-drive (default)
ros2 launch robot_sim sim.launch.py drive_mode:=ackermann  # Ackermann front-steer
```
Teleop (separate terminal):
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

## World: basic_world
- Flat ground plane + sun
- Two orange traffic cones (custom SDF — cylinder+cylinder, no mesh)
  - `traffic_cone_1` at (2.0, 0.5, 0)
  - `traffic_cone_2` at (2.0, -0.5, 0)
- Robot spawns at (0, 0, 0.05)
- `GAZEBO_MODEL_PATH` extended in launch file so Gazebo finds `model://traffic_cone`

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
- **Fix 2: wheel joint damping=0.005** (was 0.05; widened stable P range from [0.052, 0.102] to [0.027, 0.158])
- **Fix 3: PID = P=0.05, I=0.5, D=0** — D=0 avoids derivative spike on first PID call; I eliminates steady-state friction error
- **Fix 4: pause/unpause** after spawn (RegisterEventHandler in launch file) — prevents NaN from GetVelocity(0) returning uninitialized value on first physics step
- **Do NOT set D≠0** for linear velocity PID — the derivative of velocity error spikes on the first call even after pause/unpause
- Steer PID uses P=2.0, D=0 — the steer joint has damping=0.3 Nm·s/rad which overdamps naturally; no D needed
- The Prius demo uses P=800 at 100 Hz because prius wheel inertia is ~2000× larger (0.586 vs 0.00025 kg·m²)

## Checkpoints
- [x] **CP1** — Flat world, two cones, LIMO-like robot, teleop drive, RViz2
- [x] **CP2** — Dual drive modes (diff 4WD + Ackermann), active camera (30 FPS), RViz2
- [ ] **CP3** — SLAM (slam_toolbox), build a map of the environment
- [ ] **CP4** — Nav2 autonomous navigation to goal poses
- [ ] **CP5** — Computer vision integration for cone detection

## Installed Packages (relevant)
`ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-plugins`, `ros-humble-xacro`,
`ros-humble-robot-state-publisher`, `ros-humble-rviz2`, `ros-humble-teleop-twist-keyboard`,
`ros-humble-diff-drive-controller`, `ros-humble-gz-ros2-control`,
`ros-humble-ackermann-steering-controller`, `ros-humble-controller-manager`,
`ros-humble-forward-command-controller`, `python3-colcon-common-extensions`

Not yet installed (needed for CP3+): `ros-humble-slam-toolbox`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`
