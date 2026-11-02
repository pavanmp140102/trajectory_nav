^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajectory_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-11-02)
------------------
A rework after a code review of 0.0.0. The node names and topic names are unchanged.
Message QoS, parameters and config files have changed; see *Breaking changes* below.

Fixed
~~~~~
* **Controller drove back towards the start of the path.** The loader republished
  ``/raw_path`` at 10 Hz, so ``/trajectory`` arrived at 10 Hz too, and every message
  reset the controller's search to index 0. Once the robot was more than one lookahead
  distance along, the path start was picked as the target and the robot circled.
  Paths are now published once on latched topics, an identical path no longer resets
  progress, and the lookahead search runs forward from the point nearest the robot.
* **New waypoints started at the odom origin** instead of at the robot. Each set now
  begins at the robot's pose and extends along its heading.
* **Time was stored in** ``pose.position.z`` **of the trajectory**, so RViz drew the
  trajectory climbing upwards. ``z`` is now 0; each pose's ``header.stamp`` holds its
  planned arrival time.
* **The trajectory could end short of the goal.** The resampler dropped the final point
  on 16–73 % of random paths and its spacing was up to 38 % off ``sampling_distance``.
  It now resamples at exact arc-length steps and always keeps the final point.
* **Duplicate waypoints crashed the spline smoother.** They are now removed before
  fitting, and fitting errors are logged instead of raised.
* **Stale markers in RViz.** The never-used ``DELETEALL`` marker is gone; each namespace
  is now one ``SPHERE_LIST`` or ``LINE_STRIP`` with a fixed id.
* **Zero timestamps.** All messages are now stamped from the node clock, so sim time works.
* **"Trajectory complete!" and a zero command every 100 ms at the goal.** The controller
  now latches ``done`` and sends a single stop.
* **No stop command on shutdown.** Ctrl-C now sends a final zero velocity, and nodes exit
  promptly even under sim time before ``/clock`` arrives.
* **The steering law divided by the configured lookahead** even when the target was the
  closer final point. It now uses the real distance.
* **Unused imports** (``Twist``, ``PoseStamped``, ``JointTrajectory``, ``Clock``, ...).
* **The goal check lived in two nodes.** The loader and the controller each compared the
  robot to the goal with their own copy of ``goal_tolerance``, so changing it on one of them
  could leave the robot parked forever. Only the controller decides now, and it tells the
  loader on ``/goal_reached``.
* **A path received mid-route** (for example by a restarted controller) was tracked from
  the start of the path. The first update after a new path now searches the whole path for
  the robot's nearest point.
* **``min_velocity: 0`` stalled the robot** just short of the goal. It must be greater than 0.
* **NaN odometry** produced a command with a NaN turn rate. The robot now stops.
* **The final stop could be lost.** The controller waits up to 0.5 s for the zero command to
  be delivered, and SIGTERM (``kill``, ``docker stop``) is handled like Ctrl-C.
* **Parameters that cannot change at runtime looked as if they could.** They are now
  read-only, so ``ros2 param set`` on them fails.
* **``num_waypoints`` below 3** stalled the pipeline or crashed the loader. It is checked at
  startup.
* ``/odom`` is subscribed with sensor-data QoS, so best-effort odometry publishers work.
* The lookahead scan looks one window at a time instead of at every remaining path point.

Added
~~~~~
* ``/goal_reached`` (``std_msgs/Empty``): sent once by the controller when it reaches the goal.
* Slow-down near the goal: speed ramps to ``min_velocity`` at the goal tolerance.
* Turn-rate clamping that keeps the steering arc (speed is scaled down with it).
* Turn in place when the lookahead point is behind the robot.
* Stop when odometry is older than ``odom_timeout``.
* Runtime tuning of the controller with ``ros2 param set``, with validation.
* ``stamped_cmd_vel`` option: ``TwistStamped`` (default) or ``Twist`` on ``/cmd_vel``.
* ``frame_id`` and ``seed`` parameters; waypoint count and ranges as parameters.
* ``demo.launch.py``: the pipeline driving ``turtlebot3_fake_node`` with RViz, no simulator needed.
* RViz config ``rviz/trajectory_nav.rviz``; ``rviz:=true`` and ``use_sim_time`` launch arguments.
* ROS-free modules ``path_math``, ``pure_pursuit`` and ``simulation``, with real unit and
  closed-loop tests (the old tests re-implemented the maths and asserted nothing).
* Apache-2.0 headers on every file; the copyright, flake8 and pep257 tests now pass.

Changed
~~~~~~~
* Smoothing density is set by ``resolution`` (metres) instead of a fixed 100 samples.
* The yaw is computed directly from the quaternion; ``tf_transformations`` is no longer used.
* ``package.xml`` declares every runtime dependency, so ``rosdep install`` works.
* Documentation rewritten: a new README, plus Architecture, Algorithms, Parameters,
  TurtleBot3 and Testing pages with figures and RViz screenshots.

Removed
~~~~~~~
* The matplotlib plot in ``spline_smoother`` (``enable_plot``): it blocked the node until
  the window was closed. RViz shows the same information.
* ``QUICKSTART.sh`` and the overlapping ``CONFIG_FILES``, ``CONFIG_SIMPLE`` and
  ``QUICKSTART`` documents.

Breaking changes
~~~~~~~~~~~~~~~~
* The path topics use reliable, **transient local**, depth 1 QoS. Subscribers that need
  the latest path when they start must request transient local durability.
* A publisher that feeds ``/raw_path``, ``/smoothed_path`` or ``/trajectory`` from outside
  must also use transient local durability; volatile publishers are not delivered.
* ``goal_tolerance`` is now a controller parameter only; the loader no longer has one.
* ``min_velocity`` must be greater than 0.
* The three per-node config files are replaced by one ``config/trajectory_nav.yaml``.
* Controller parameter ``linear_velocity`` is now the shared ``velocity``.
* ``spline_smoother`` parameter ``enable_plot`` is removed.

0.0.0 (2026-03-15)
------------------
First version.

* Five nodes: ``waypoint_loader``, ``spline_smoother``, ``trajectory_generator``,
  ``pure_pursuit_controller`` and ``trajectory_visualizer``.
* Random waypoints from the odom origin, republished at 10 Hz and regenerated when the
  robot comes within 0.3 m of the last one.
* Arc-length natural cubic spline (SciPy), resampled to 100 points, with an optional
  matplotlib plot.
* Trajectory resampled at ``sampling_distance`` with constant-velocity timestamps stored
  in ``pose.position.z`` and ``header.stamp``.
* Pure pursuit with a fixed lookahead and constant speed, stopping within 0.2 m of the end;
  ``TwistStamped`` on ``/cmd_vel``.
* RViz markers for the raw, smoothed and trajectory paths.
* Per-node YAML configs and ``trajectory_nav.launch.py``.
