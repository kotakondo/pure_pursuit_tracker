# pure_pursuit_tracker

Pure pursuit path tracking controller with feedforward curvature for ROS 2 ground robots. Runs at 20 Hz with adaptive lookahead, curvature-based speed limiting, and turn-in-place for large heading errors.

## Dependencies

- ROS 2 Humble
- `rclcpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2`, `tf2_geometry_msgs`

## Build

```bash
cd ~/trajectory_generator_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pure_pursuit_tracker
source install/setup.bash
```

## Run

```bash
ros2 launch pure_pursuit_tracker pure_pursuit.launch.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_name` | `$ROVER_NAME` or `RR03` | Namespace for all topics |
| `odom_topic` | `odom` | Input odometry topic (relative to namespace) |
| `cmd_vel_topic` | `cmd_vel` | Output velocity command topic (relative to namespace) |

### Topic remapping examples

```bash
# Custom namespace
ros2 launch pure_pursuit_tracker pure_pursuit.launch.py robot_name:=ROVER1

# Remap odom and cmd_vel
ros2 launch pure_pursuit_tracker pure_pursuit.launch.py odom_topic:=odom_filtered cmd_vel_topic:=cmd_vel_auto
```

## Subscribed topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `reference_trajectory` | `nav_msgs/msg/Path` | reliable + transient_local | Reference path from trajectory_generator |
| `odom` | `nav_msgs/msg/Odometry` | default | Robot odometry (remappable via `odom_topic`) |

## Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands (remappable via `cmd_vel_topic`) |
| `actual_trajectory` | `nav_msgs/msg/Path` | Accumulated robot path (never cleared) |
| `lookahead_point` | `geometry_msgs/msg/PointStamped` | Current lookahead target |
| `lookahead_marker` | `visualization_msgs/msg/Marker` | RViz lookahead visualization (yellow sphere) |

## Parameters

Configured via `config/params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookahead_distance` | `0.4` | Minimum lookahead distance (m) |
| `k_v` | `0.3` | Velocity-dependent lookahead gain |
| `max_linear_velocity` | `0.5` | Max forward speed (m/s) |
| `max_angular_velocity` | `1.5` | Max turn rate (rad/s) |
| `goal_tolerance` | `0.3` | Distance to goal to declare reached (m) |
| `control_frequency` | `20.0` | Control loop rate (Hz) |
| `adaptive_lookahead_distance` | `2.0` | Goal approach taper distance (m) |
| `turn_in_place_threshold_deg` | `60.0` | Heading error for turn-in-place (deg) |
| `slow_down_threshold_deg` | `30.0` | Heading error to start slowing (deg) |
| `w_smoothing_alpha` | `0.15` | Angular velocity smoothing (0=none, 1=full) |
| `max_lateral_acceleration` | `0.3` | Lateral accel limit for curves (m/s^2) |

## Logging

Each run writes a CSV log to `/tmp/pure_pursuit_log.csv` with columns: time, closest_idx, ref/robot positions, yaw, alpha, curvature, velocity/angular commands, cross-track error, feedforward/pursuit/heading angular terms.
