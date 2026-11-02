# Testing

## Running the tests

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select trajectory_nav
colcon test --packages-select trajectory_nav
colcon test-result --verbose
```

Expected result: `62 tests, 0 errors, 0 failures, 0 skipped`.

The maths tests don't need ROS running, so you can also run them directly while you work:

```bash
cd src/trajectory_nav
python3 -m pytest test/test_path_math.py test/test_pure_pursuit.py -v
```

## What is covered

### `test_path_math.py`: path maths

| Test | Checks |
|---|---|
| waypoints start at robot | first waypoint = robot position; later ones move forward along the heading |
| waypoints reproducible | the same seed gives the same waypoints |
| waypoint count | fewer than 3 waypoints is rejected up front |
| positive numbers | zero, negatives, NaN and infinity are rejected by `require_positive` |
| remove close points | duplicates and near-duplicates are dropped |
| smoothing passes through waypoints | over 50 random sets: endpoints kept, every waypoint on the curve |
| smoothing density | sample spacing follows `resolution`, not a fixed count |
| duplicate waypoints | no crash on repeated points |
| too few points | a clear `ValueError` the node can catch |
| resample endpoint and spacing | 4 spacings × 20 paths: start and end kept exactly, every step exactly `spacing` along the path |
| timing, headings, yaw ↔ quaternion, angle wrap | basic conversions |

### `test_pure_pursuit.py`: the controller

| Test | Checks |
|---|---|
| repeated path keeps progress | the same path sent again doesn't move the target behind the robot |
| new path resets progress | a different path restarts tracking and clears `done` |
| straight line | full speed, no turning |
| turns towards path | correct sign of ω |
| real distance to target | ω uses the actual distance near the end, not the lookahead |
| slow-down | speed ramps down near the goal and stays ≥ `min_velocity` |
| target behind | turns in place |
| clamping keeps curvature | v scales with ω, so the arc stays the same |
| stop and latch | stops inside the tolerance and stays stopped |
| invalid tuning | 13 bad values (zero, negative, NaN, infinity) each raise `ValueError`; an unknown name raises `TypeError` |
| path received mid-route | a fresh tracker starts at the robot's nearest point, over 30 random paths, and from a mid-path start finishes without going back |
| non-finite pose | NaN or infinite odometry gives a stop, not a NaN command |
| long path | the windowed lookahead scan finds the same point as a full scan |
| sparse path | paths with points 0.05 to 3 m apart are all followed to the goal |
| passed points | the lookahead never picks a point the robot has already driven past |
| search window | every window holds at least two points, so sparse paths can be stepped through |
| non-finite path | a path with NaN or infinity is rejected |
| closed loop, 20 random paths | the simulated robot reaches every goal and stays within 0.6 m of the path |
| closed loop, path re-sent every step | the old 10 Hz republish no longer breaks tracking |

Randomised tests use a fixed seed, so a failure always reproduces.

### Lint tests

`test_flake8.py`, `test_pep257.py` and `test_copyright.py` run the ament linters over the package. Every source file carries the Apache-2.0 header.

## Manual end-to-end check

The ROS wiring (QoS, launch files, message conversion) is checked by running the demo:

```bash
ros2 launch trajectory_nav demo.launch.py rviz:=false
```

Within about 45 s the log should show `Goal reached`, then `Published 6 waypoints` starting at the robot's position, then `Tracking new trajectory`. Ctrl-C should end every process with `process has finished cleanly`.

The RViz screenshots in these docs come from this demo with `seed: 3`, so the same paths appear on every run.
