# Indoor Cartographer Mapping Demo

This demo adds a high-detail Gazebo apartment scene for Cartographer 2D SLAM.
The layout contains two bedrooms, one kitchen, one living room, and one
bathroom. The robot has a 2D lidar, wheel odometry, and an RGB camera published
as `/robot_view/image_raw`.

Run it from `~/cartographer`:

```bash
./run_indoor_mapping_demo.sh
```

The detailed apartment remains the default. Use the smaller, wider quick home
when iterating on exploration and recovery behavior:

```bash
./run_indoor_mapping_demo.sh scene:=quick
./run_indoor_mapping_demo.sh scene:=detailed
```

The quick scene starts with a moving kitten obstacle by default. The kitten is
spawned as a Gazebo model with collision geometry, so the 2D lidar sees it while
Cartographer and the active explorer continue mapping the static apartment:

```bash
./run_indoor_mapping_demo.sh scene:=quick dynamic_obstacles:=false
```

A YOLO cat detector is enabled by default for the robot camera. RViz's lower-left
`Robot View` panel displays `/robot_view/yolo/image_annotated`, drawing `cat`
boxes and confidence values when the moving kitten is visible:

```bash
./run_indoor_mapping_demo.sh scene:=quick yolo_cat_detector:=false
./run_indoor_mapping_demo.sh scene:=quick yolo_confidence:=0.20
```

Useful launch arguments:

```bash
./run_indoor_mapping_demo.sh autonomous:=false
./run_indoor_mapping_demo.sh gui:=false
./run_indoor_mapping_demo.sh rviz:=false
```

This branch also includes an optional 3D Cartographer path. It keeps the 2D
demo as the default, and enables a simulated IMU plus a block-laser point cloud
only when `slam_mode:=3d` is selected:

```bash
./run_indoor_mapping_demo.sh scene:=quick slam_mode:=3d
./run_indoor_mapping_demo.sh scene:=quick slam_mode:=3d gui:=false rviz:=false
```

The 3D mode publishes `/imu_raw`, planar-stabilized `/imu`, `/block_laser_3d`,
and filtered `sensor_msgs/PointCloud2` data on `/points2` for Cartographer. The
block-laser stream is range and height filtered for the quick indoor scene so
max-range returns, floor hits, and ceiling-like shell noise do not dominate the
map. Active frontier exploration drives the robot unless `autonomous:=false`
is selected.
When `slam_mode:=3d` is selected, RViz opens `indoor_mapping_3d.rviz`: it uses
an orbit camera and enables Cartographer submaps, trajectory markers, and the
voxel-downsampled accumulated 3D point map on `/map_points`. The raw live 3D
scan remains available on `/points2`. The `/map` topic is still available, but
it is only a 2D occupancy-grid projection of the 3D run.

The recommended 3D reconstruction mode is the hybrid pipeline. It uses stable
Cartographer 2D SLAM for autonomous exploration and feeds the independent 3D
block laser into OctoMap:

```bash
./run_indoor_mapping_demo.sh scene:=quick slam_mode:=hybrid
./run_indoor_mapping_demo.sh scene:=quick slam_mode:=hybrid gui:=false rviz:=false
```

Hybrid mode publishes the navigation map on `/map`, the filtered 3D scan on
`/octomap/points_filtered`, the binary tree on `/octomap_binary`, and occupied
voxels on `/occupied_cells_vis_array` and `/octomap_point_cloud_centers`.
`octomap_cloud_filter.py` removes distant returns, downsamples each scan,
filters points around the moving kitten using Gazebo link poses, and stops
publishing after `/active_slam/completed=True`. The OctoMap frame is `odom`, so
2D pose-graph corrections move the complete 3D map coherently instead of
smearing scans that were accumulated before optimization. The hybrid RViz
profile hides raw scan and debug overlays by default.

RViz opens with a narrow left display panel, the `Robot View` camera panel
docked at the lower left, and a larger map view. The display stack includes the
Cartographer map/submaps, scan, robot model, Active SLAM frontiers, the active
path, the current target, and the lower-left camera view.

Autonomous exploration now defaults to a lightweight Active SLAM node instead
of `move_base + explore_lite`. `active_slam_explorer.py` reads `/map`, `/scan`,
and TF, runs reachable-frontier detection plus cost-aware Dijkstra planning in
known free space, follows a lookahead
point on that path, and falls back to fast open-space probing when the initial
frontiers are too close. Once a frontier is reached, a directed deep-probe
phase continues toward nearby unknown cells instead of immediately selecting a
different room. This keeps the loop small and responsive while improving the
coverage of narrow corridors and room ends.

The default profile is tuned for fast coverage: control commands are published
at `24 Hz`, the cruise speed is `0.82 m/s`, and frontier scoring favors larger
unknown regions over tiny nearby frontiers. Local obstacle handling uses a
VFH-style laser heading selector, so each control step chooses a safe gap that
still points toward the active frontier whenever possible.

The local controller evaluates a footprint-width laser corridor rather than a
single center ray. A Gazebo contact sensor provides a final collision signal.
When motion is blocked, recovery brakes, checks rear clearance, backs away,
turns toward the more open side, probes forward, blacklists the failed frontier,
and forces a fresh global frontier plan.

The compact robot is approximately `0.28 m` long and `0.28 m` wide across the
wheels. Its sky-blue wings are visual-only links without collision geometry, so
they do not reduce access to narrow passages. Planning uses a graded costmap on `/active_slam/costmap`: lethal cells
cover the physical footprint, while a wider decaying cost band keeps Dijkstra
paths away from walls. RViz shows it as `Navigation Costmap` with low alpha so
the occupancy map remains readable. Cartographer submaps, trajectory nodes, and
constraints remain available but disabled by default to avoid visual clutter.

Exploration completion requires repeated plans with no useful reachable
frontier, a minimum mapping runtime, a minimum accumulated travel distance, and
a stable known-cell count. Each bundled scene also has a minimum known-cell
coverage gate, preventing a visually open apartment map from being accepted
just because exploration temporarily stalls. Low-value patrol candidates
cannot postpone return forever once those coverage gates are satisfied. Patrol
scoring only counts unknown cells with a
wall-free line of sight, so unknown space outside closed exterior walls cannot
cause endless patrol while unfinished corridors and rooms remain eligible.
After broad coverage, a closure-scan phase measures unknown lidar shadows
inside each bundled scene's known wall bounds and selects reachable viewpoints
that can see them directly. Using configured wall bounds prevents a missing
outer wall from shrinking the measured interior and causing false completion.
Recently used closure viewpoints are penalized so wall
corners and furniture occlusions are observed again from a different angle.
Recorded visit positions prefer areas that have seen less coverage. The robot
then plans back to its recorded start position. At home it publishes
`/active_slam/completed=True` and continues publishing a zero `/cmd_vel`.
It also finishes the Cartographer trajectory so the pose graph performs its
final loop-closure optimization. Dead ends trigger a longer checked reverse
before turning and replanning.

If normal frontier clusters temporarily disappear, the explorer now selects a
reachable patrol waypoint near unknown space and follows a BFS path instead of
rotating indefinitely. Target hysteresis prevents rapid left/right goal changes.
The controller also detects high accumulated rotation with little translation
and forces recovery. Map access is synchronized so occupancy-grid expansion
cannot terminate the control timer and leave Gazebo executing a stale command.

Cartographer receives strictly increasing odometry through
`/cartographer/odom`; duplicate Gazebo timestamps are removed by
`indoor_odom_filter.py`. RViz includes Cartographer submaps and trajectory nodes,
with the denser constraint visualization available but disabled by default.

Main Active SLAM topics:

```bash
/active_slam/status
/active_slam/frontiers
/active_slam/path
/active_slam/target
```

The old navigation stack is still available for comparison:

```bash
./run_indoor_mapping_demo.sh active_slam:=false navigation:=true explore:=true
```

The active costmap is the `Navigation Costmap` display. The legacy move_base
local costmap remains disabled and is only relevant when launching the optional
navigation stack.
