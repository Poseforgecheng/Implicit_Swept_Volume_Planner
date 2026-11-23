# Benchmark task files

Task files are generated with `generate_tasks.py` or written manually using the
schema documented in the package README. No RViz clicks are used by the runner.

The following hand-authored files provide two key tasks for every map currently
shipped under `planner_experiments/maps/`:

| Map directory | Task file | Intended challenge |
|---|---|---|
| `bench_dual_route` | `bench_dual_route.yaml` | symmetric upper/lower alternatives |
| `bench_l_corner_180` | `bench_l_corner_180.yaml` | 90-degree footprint turn |
| `bench_narrow_door_180` | `bench_narrow_door_180.yaml` | centered narrow-door crossing |
| `bench_random_obstacles_seed1` | `bench_random_obstacles_seed1.yaml` | two cluttered diagonal crossings |
| `bench_rotation_channel_180` | `bench_rotation_channel_180.yaml` | continuous yaw adaptation |
| `bench_s_channel_180` | `bench_s_channel_180.yaml` | alternating top/bottom passages |
| `ditu` | `ditu.yaml` | imported large-building crossings |
| `smoke_empty` | `smoke_empty.yaml` | 0.10 m empty-map control |
| `smoke_empty_005` | `smoke_empty_005.yaml` | 0.05 m empty-map control |

All files bind `long_rectangle` by default and use metres/radians.  Their
start/goal footprints were densely checked at 0.01 m against the map.  The same
endpoint poses also pass the endpoint check for all four shipped footprints,
but a task file must still reference the matching robot YAML when changing
shape.

`smoke.yaml` remains as the backwards-compatible task file used by existing
README commands; it describes the same `smoke_empty` control map.

Run every hand-authored map task through the same method set with:

```bash
python3 planner_experiments/scripts/run_batch.py \
  --config planner_experiments/config/experiment_default.yaml \
  --datasets planner_experiments/datasets/all_hand_authored_map_tasks.yaml \
  --output results/all_hand_authored_map_tasks \
  --parallel-workers 1 \
  --postprocess
```
