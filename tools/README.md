# Passive planning-evidence tools

These scripts consume existing ROS topics. They do not start a chassis, publish
motion commands, stop the robot, or change the planner method.

From the repository root, after sourcing ROS and the built workspace:

```bash
python3 tools/scripts/record_planning_evidence.py \
  --mode isweep_v3 --output results/physical/session_001
```

Choose a **new** output directory. Start recording before sending a new goal.
The additional `evidence.bag` preserves messages alongside CSV/JSON by default.
Use `--no-bag` only when bag recording is intentionally unnecessary. `--help`
lists pose, odometry, goal, and map topic overrides and recording tolerances.
`--timeout` is an observation annotation, not a chassis-stop watchdog.

`experiment_recorder.py` provides the per-run CSV and offline-bag summary
interface. Run it with the ROS system Python and use `--help` for options.
For IR-SIM, remap physical localization, command, and scan inputs to simulator
topics rather than mixing physical and simulated states.

```bash
python3 -m unittest discover -s tools/test -v
```

The ROS serialization test requires the generated `isweep_planner` messages
from the sourced devel space. Pure storage/geometry tests need no ROS master.
Tracking errors use the selected localization stream, not motion-capture truth.
