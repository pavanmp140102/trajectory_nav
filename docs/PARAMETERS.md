# Parameters

All nodes read [`config/trajectory_nav.yaml`](../src/trajectory_nav/config/trajectory_nav.yaml). Parameters under `/**` apply to every node, so values that must agree, such as the speed, are set once.

## Shared (`/**`)

| Parameter | Default | Used by | Meaning |
|---|---|---|---|
| `frame_id` | `odom` | loader, smoother, generator | Frame of the published paths |
| `velocity` | 0.3 m/s | generator, controller | Cruise speed: trajectory timing and controller speed |

## waypoint_loader

| Parameter | Default | Meaning |
|---|---|---|
| `num_waypoints` | 6 | Waypoints per set, including the first one at the robot (at least 3) |
| `step_min` | 1.0 m | Minimum forward spacing between waypoints |
| `step_max` | 2.0 m | Maximum forward spacing between waypoints |
| `lateral_range` | 2.0 m | Maximum sideways offset from the robot's heading |
| `seed` | -1 | Random seed. -1 gives different paths every run; any other value repeats the same sequence |

## spline_smoother

| Parameter | Default | Meaning |
|---|---|---|
| `resolution` | 0.05 m | Spacing of the smoothed path samples |

## trajectory_generator

| Parameter | Default | Meaning |
|---|---|---|
| `sampling_distance` | 0.05 m | Spacing of the trajectory points along the path (must be > 0) |

## pure_pursuit_controller

| Parameter | Default | Runtime | Meaning |
|---|---|---|---|
| `lookahead_distance` | 0.5 m | yes | Steer towards the path point this far ahead |
| `goal_tolerance` | 0.2 m | yes | Goal reached within this distance of the last point; the controller stops and announces the goal on `/goal_reached` |
| `max_angular_velocity` | 1.0 rad/s | yes | Turn-rate limit; speed is reduced with it so the robot keeps the same arc |
| `slowdown_distance` | 0.5 m | yes | Start ramping speed down this far from the goal; 0 disables the ramp |
| `min_velocity` | 0.05 m/s | yes | Speed when arriving at `goal_tolerance`; must be greater than 0 so the robot can reach the goal |
| `velocity` | 0.3 m/s | yes | Cruise speed (shared) |
| `odom_timeout` | 0.5 s | no | Stop if odometry is older than this |
| `control_frequency` | 20 Hz | no | Control loop rate |
| `goal_retry_period` | 2.0 s | no | After the goal, how long to wait for a new trajectory before warning and sending `/goal_reached` again |
| `stamped_cmd_vel` | true | no | `TwistStamped` on `/cmd_vel`; false for `Twist`. Set by the launch argument of the same name |

Parameters marked **Runtime** can be changed while the controller runs. Invalid values are rejected and the old ones are kept. Every other parameter, in every node, is read once at startup and is **read-only**: `ros2 param set` on it fails instead of appearing to work.

```bash
ros2 param set /pure_pursuit_controller lookahead_distance 0.7
ros2 param set /pure_pursuit_controller lookahead_distance 0.0  # rejected: must be positive
```

Changing `velocity` at runtime affects only the controller. The trajectory timestamps keep using the speed the generator started with.

These limits are enforced: lookahead, velocity, turn rate and goal tolerance must be positive, `slowdown_distance` must not be negative, and `min_velocity` must be greater than 0 and at most `velocity`. At startup, `num_waypoints` must be at least 3 (the spline smoother needs three points), `0 < step_min <= step_max`, and `lateral_range`, `resolution`, `sampling_distance`, `odom_timeout`, `control_frequency` and `goal_retry_period` must be valid, or the node exits with an error. "Valid" means a finite number: NaN and infinity are rejected everywhere, at startup and at runtime.

## Launch arguments

### trajectory_nav.launch.py

| Argument | Default | Meaning |
|---|---|---|
| `params_file` | installed `config/trajectory_nav.yaml` | Parameter file for all nodes |
| `use_sim_time` | `false` | Set `true` with Gazebo |
| `rviz` | `false` | Start RViz with `rviz/trajectory_nav.rviz` |
| `stamped_cmd_vel` | `true` | `TwistStamped` (TurtleBot3 Jazzy, Gazebo) or `Twist` |

### demo.launch.py

| Argument | Default | Meaning |
|---|---|---|
| `params_file` | installed `config/trajectory_nav.yaml` | Parameter file for all nodes |
| `rviz` | `true` | Start RViz |

The demo always uses plain `Twist`, because `turtlebot3_fake_node` subscribes to `geometry_msgs/Twist`.

## Using your own parameter file

Copy the installed file, edit it, and pass it in:

```bash
cp $(ros2 pkg prefix trajectory_nav)/share/trajectory_nav/config/trajectory_nav.yaml ~/my_params.yaml
ros2 launch trajectory_nav demo.launch.py params_file:=$HOME/my_params.yaml
```

## Tuning tips

- **The robot cuts corners:** shorten `lookahead_distance`. Measured errors for different values are in [ALGORITHMS.md](ALGORITHMS.md#tuning-the-lookahead).
- **The robot weaves on a real base:** lengthen `lookahead_distance` or lower `velocity`.
- **The robot stops short or overshoots the goal:** `goal_tolerance` sets where it stops, and `slowdown_distance` and `min_velocity` set how gently.
- **Paths are too twisty for the robot:** lower `lateral_range` or raise `step_min`.
- **You need repeatable runs** for tests or screenshots: set `seed` to any value ≥ 0.
