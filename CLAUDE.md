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
      launch/sim.launch.py     ← main entry point
      worlds/basic_world.world
      models/traffic_cone/     ← custom Gazebo model (no system cone exists)
      urdf/limo_sim.urdf.xacro ← LIMO-like differential drive robot
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
ros2 launch robot_sim sim.launch.py
```
Teleop (separate terminal):
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Robot: limo_sim
- Differential drive, LIMO AGILEX proportions
- Chassis: 0.4×0.3×0.12m, gray, 4kg
- Drive wheels: radius=0.045m, track width=0.17m (y=±0.085 from base_link)
- Front casters: sphere radius=0.025m, frictionless (mu1=0.001)
- TF tree: `base_footprint` → `base_link` → wheels, casters, camera_link, lidar_link
- Plugin: `libgazebo_ros_diff_drive.so` (Gazebo Classic built-in, no ros2_control)
- Plugin: `libgazebo_ros_joint_state_publisher.so` → publishes `/joint_states`
- Topics: `/cmd_vel` (in), `/odom` (out), `/joint_states` (out)
- `camera_link` and `lidar_link` are geometry-only placeholders — no active sensors yet

## World: basic_world
- Flat ground plane + sun
- Two orange traffic cones (custom SDF model — cylinder+cylinder, no mesh)
  - `traffic_cone_1` at (2.0, 0.5, 0)
  - `traffic_cone_2` at (2.0, -0.5, 0)
- Robot spawns at (0, 0, 0.01), settles onto ground at sim start
- `GAZEBO_MODEL_PATH` extended in launch file + package.xml export so Gazebo finds `model://traffic_cone`

## Key Technical Decisions
- **Gazebo Classic 11, not Ignition**: better LIMO package compatibility long-term
- **No ros2_control at checkpoint 1**: using simpler `libgazebo_ros_diff_drive.so` directly
- **`ParameterValue(..., value_type=str)` required** around `Command(['xacro', ...])` in launch files — ROS2 Humble parses Command output as YAML otherwise and crashes
- **`--symlink-install`** means editing source files in `src/` takes effect without rebuilding

## Checkpoints
- [x] **CP1** — Flat world, two cones, LIMO-like robot, teleop drive, RViz2
- [ ] **CP2** — Add active camera sensor + lidar, visualize in RViz2
- [ ] **CP3** — SLAM (slam_toolbox), build a map of the environment
- [ ] **CP4** — Nav2 autonomous navigation to goal poses
- [ ] **CP5** — Computer vision integration for cone detection

## Installed Packages (relevant)
`ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-plugins`, `ros-humble-xacro`,
`ros-humble-robot-state-publisher`, `ros-humble-rviz2`, `ros-humble-teleop-twist-keyboard`,
`ros-humble-diff-drive-controller`, `ros-humble-gz-ros2-control`, `python3-colcon-common-extensions`

Not yet installed (needed for CP3+): `ros-humble-slam-toolbox`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`
