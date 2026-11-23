# iSweep V3 closed-loop validation in IR-SIM

Standalone repository notes: this repository ships `isweep_planner`,
`navigation_adapter`, and `planner_bridge`. Hardware drivers, TEB, and the
`sentry_nav` startup package are external. From this repository root, install
with `./irsim_bridge/scripts/setup_irsim_env.sh`, then build all packages from
the parent catkin workspace. The default launch map is `map.yaml`; pass
`map_file:=$(rospack find isweep_planner)/maps/office.yaml` for the office case.
The passive recorders are available as `tools/scripts/experiment_recorder.py`
and `tools/scripts/record_planning_evidence.py`; no `sentry_nav` package is
needed to run those copies with the ROS Python interpreter.

This package keeps one authoritative simulation state:

```
IR-SIM map/plant/collision truth
    -> ROS map, localization, odometry, LiDAR and TF
    -> navigation_adapter (same start/goal/replan contract as the robot)
    -> iSweep V3 planning and local control
    -> planner_bridge Ackermann gate
    -> Twist-to-steering conversion
    -> IR-SIM Ackermann plant [linear velocity, steering angle]
    -> RViz planned/executed trajectory display
```

Gazebo is deliberately not part of this loop. It can be added later as a
separate plant experiment; running two plants at once would make collision and
vehicle state attribution ambiguous.

## 1. Install the isolated IR-SIM runtime

ROS Noetic uses Python 3.8 on this machine, while IR-SIM 2.10.2 requires Python
3.10 or newer. The setup script creates a package-local virtual environment and
does not modify the existing `nav` conda environment:

```bash
cd ~/isweep_ws/src/Implicit_Swept_Volume_Planner/irsim_bridge
./scripts/setup_irsim_env.sh
```

The script selects a Python >=3.10 automatically. Override it when necessary:

```bash
IRSIM_BOOTSTRAP_PYTHON=/path/to/python3.11 ./scripts/setup_irsim_env.sh
```

## 2. Build

```bash
cd ~/isweep_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg navigation_adapter isweep_planner planner_bridge irsim_bridge
source devel/setup.bash
```

## 3. Start the closed loop

The default case uses the 10 m x 10 m `office` map and a rectangular Ackermann
robot matching the current 0.45 m x 0.36 m iSweep footprint:

```bash
roslaunch irsim_bridge isweep_v3_irsim_rviz.launch
```

RViz colors are fixed across runs:

- gray: front-end candidate path;
- red: validated iSweep trajectory;
- blue: local tracking trajectory;
- green: trajectory actually executed by IR-SIM.

Use **2D Pose Estimate** to reset the IR-SIM vehicle and **2D Nav Goal** to
replace the goal and trigger a new iSweep plan. The `Reset` service restores the
launch-file start pose:

```bash
rosservice call /irsim/reset
```

The local tracker switches to a low-speed terminal Ackermann controller near
the goal. A run is marked arrived only when both the configured 0.10 m position
tolerance and 0.20 rad yaw tolerance pass; collision truth is evaluated by
IR-SIM independently. This terminal controller is enabled by this simulation
launch only; the shared real-robot default remains disabled until a low-speed
chassis test explicitly enables it.

## 4. Select another ROS map and case

IR-SIM and iSweep read the same ROS map YAML. The bridge converts unknown cells
to occupied cells by default and writes the exact same binary collision grid to
IR-SIM, avoiding a simulator/planner map mismatch.

```bash
roslaunch irsim_bridge isweep_v3_irsim_rviz.launch \
    map_file:=$(rospack find isweep_planner)/maps/map_clean_nav.yaml \
    start_x:=3.16 start_y:=45.92 start_yaw:=-1.98 \
    goal_x:=5.01 goal_y:=11.73 goal_yaw:=-0.14
```

Before a formal run, confirm that both poses are in free space. For other robot
dimensions, change all geometry arguments together so IR-SIM, Hybrid A* and the
iSweep footprint configuration remain consistent.

## 5. Record a reproducible run

The simulation topics are isolated from the real chassis. Remap them only into
the existing experiment recorder:

```bash
mkdir -p ~/robot3/experiment_data/irsim_v3

python3 "$(rospack find isweep_planner)/../tools/scripts/experiment_recorder.py" \
    /localization:=/irsim/localization \
    /cmd_vel:=/irsim/cmd_vel \
    /scan:=/irsim/scan \
    --mode isweep_v3 \
    --timeout 120 \
    --position-tolerance 0.10 \
    --yaw-tolerance 0.20 \
    --dwell-time 1.0 \
    --output ~/robot3/experiment_data/irsim_v3/runs.csv
```

Record the evidence stream in a second terminal:

```bash
rosbag record -O ~/robot3/experiment_data/irsim_v3/run_001.bag \
    /prior_map /isweep_planner/map /tf /tf_static \
    /initialpose /move_base_simple/goal \
    /isweep_planner/trajectory /isweep_planner/trajectory_astar \
    /isweep_planner/local_trajectory /isweep_planner/local_cmd \
    /isweep_planner/local_planner_status /isweep_planner/replan_request \
    /isweep_planner/markers /isweep_planner/planning_time \
    /isweep_planner/planning_stats \
    /isweep_planner/risk_aware_global_reference \
    /irsim/odom /irsim/localization /irsim/cmd_vel /irsim/scan \
    /irsim/actual_path /irsim/status /irsim/collision /irsim/arrived \
    /irsim/metadata
```

`/irsim/metadata` contains the seed, step time, robot dimensions and collision
map SHA-256. `/irsim/status` contains the current state, goal errors, collision
truth, goal-attainment truth, requested/applied motion and actual vehicle
velocity after IR-SIM acceleration and steering limits.

## Primary runtime topics

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| IR-SIM -> adapter | `/prior_map` | `nav_msgs/OccupancyGrid` | Authoritative binary collision grid |
| adapter -> planner | `/isweep_planner/map` | `nav_msgs/OccupancyGrid` | Same grid under the real-robot planner contract |
| IR-SIM -> planner | `/isweep_planner/current_pose` | `geometry_msgs/PoseStamped` | Simulator ground-truth pose |
| IR-SIM -> planner | `/isweep_planner/current_velocity` | `geometry_msgs/TwistStamped` | Applied vehicle velocity |
| planner -> gate | `/isweep_planner/local_cmd` | `geometry_msgs/Twist` | iSweep local control |
| planner -> audit | `/isweep_planner/local_planner_status` | `std_msgs/String` | Local mode, result and blocking reason |
| planner -> adapter | `/isweep_planner/replan_request` | `std_msgs/Bool` | Rising-edge replan request |
| gate -> IR-SIM | `/irsim/cmd_vel` | `geometry_msgs/Twist` | Radius-limited `[v, yaw-rate]` command; the bridge converts it to `[v, steering-angle]` |
| IR-SIM -> RViz | `/irsim/actual_path` | `nav_msgs/Path` | Executed trajectory |
| IR-SIM -> recorder | `/irsim/scan` | `sensor_msgs/LaserScan` | Simulated 2D LiDAR |
| IR-SIM -> audit | `/irsim/status` | `std_msgs/String` | JSON status and terminal truth |

## 6. Compare the four paper-facing planners

The launch file accepts one audited planner method:

```text
current_full  full_r2  full_strict  hybrid_astar
```

For one interactive method, add `planner_method:=...` to the launch command.
The same argument is available from the regular RViz demo:

```bash
roslaunch isweep_planner demo.launch \
    enable_three_level_planner:=true \
    planner_method:=full_strict
```

For the Ackermann headless startup chain, set the method explicitly in the
environment (the default is `current_full`):

```bash
PLANNER_METHOD=current_full \
  rosrun sentry_nav start_navigation_headless_test.sh isweep_v3
```

For a controlled comparison, run all methods sequentially in isolated IR-SIM
world instances. The runner fixes the map, start/goal pose, simulator seed,
vehicle, controller, validation tolerances, and timeout across methods. A failed
planner remains in `summary.csv` instead of being silently removed.

```bash
mkdir -p ~/robot3/experiment_data/irsim_comparison

rosrun irsim_bridge run_planner_comparison.py \
    --map-file $(rospack find isweep_planner)/maps/serpentine_corridor.yaml \
    --start START_X START_Y START_YAW \
    --goal GOAL_X GOAL_Y GOAL_YAW \
    --seed 7 \
    --timeout 120 \
    --output ~/robot3/experiment_data/irsim_comparison/serpentine_case01
```

Each method directory contains the first validated trajectory (`planned.csv`),
the latest planner publication, the front-end/A* diagnostic candidate
(`coarse.csv`), the executed path, final status, simulator metadata, planning
statistics, `local_status.json`, roslaunch log, and a rosbag evidence stream.
The runner also writes `risk_reference.csv`; this is the exact published
LOW/MEDIUM/HIGH reference used by the local planner, not a risk label inferred
later from the plotted geometry.
The summary also reports the last local-planner status and the number of
rising-edge replan requests. `coarse.csv` is not a
validated executable trajectory and is never counted as a successful run. Use
`--no-bag` only for a quick smoke test.

The IR-SIM comparison uses an obstacle-aware reverse-grid heuristic for Hybrid
A*. Its search-only defaults are 300000 expansions, 30000 fast-pass
expansions, 7 steering samples and heuristic weight 2.0. They can be audited or
overridden with `--hybrid-max-expansions`,
`--hybrid-fast-pass-max-expansions`, `--hybrid-steer-samples`, and
`--hybrid-heuristic-weight`. Footprint collision checks and final validation
remain unchanged. A Hybrid A* chain that reaches the goal but fails continuous
trajectory validation is retained only as an unvalidated candidate.

`full_strict` forces every optimization segment through the full SVSDF solver,
but its published local-control reference retains the independently assessed
LOW/MEDIUM/HIGH geometric labels. This keeps solver ablation separate from the
risk-aware tracking policy.

Generate the paper-ready comparison after the run. Each method receives one
panel, with its planned path (dashed) and executed path (solid) overlaid in the
same map coordinates. If final validation fails, the panel retains the
available front-end/A* candidate as a gray dash-dot line labelled
`Unvalidated candidate`; for a failed Hybrid A* search, this can be the nearest
partial search chain. If no candidate was found, the panel says so explicitly:

```bash
rosrun irsim_bridge plot_planner_comparison.py \
    ~/robot3/experiment_data/irsim_comparison/serpentine_case01
```

The output directory receives editable SVG, vector PDF, and 300 dpi PNG files.
For result directories produced before `coarse.csv` was added, the plotter
recovers the last non-empty `/isweep_planner/trajectory_astar` message from
`evidence.bag` when available, so those runs can normally be re-plotted without
rerunning the planners.
