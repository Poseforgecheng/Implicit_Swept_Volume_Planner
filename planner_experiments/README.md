# planner_experiments

`planner_experiments` 是 `isweep_planner` 的二维离线批量实验层。它直接调用仓库现有的 `SvsdfRuntime`、R² 优化、strict SE(2)、完整 SVSDF/LMBM、Swept A* 和 Hybrid A*，不包含替代或模拟规划器。Gazebo、实车和人工 RViz 点击均不在执行链路中。

本发布版的离线实验读取 `isweep_planner/config/planner_benchmark.yaml`，与交互/实车的矩形配置及平滑参数分开。以下命令假定仓库位于 `~/isweep_ws/src/Implicit_Swept_Volume_Planner`。本次发布更新了规划核心，不能将新运行直接标记为历史论文实验的原始记录。

## 架构

```text
YAML task + ROS map + polygon/mesh
              |
      GeometryConsistencyChecker
              |
         PlannerAdapter
              |
     existing SvsdfRuntime::Plan
              |
 independent TrajectoryValidator
              |
 atomic per-run result -> batch JSONL/CSV -> checks/analysis/plots
```

每个组合在独立进程中运行。进程硬超时、配置错误、无路径或验证失败均写入结果后继续下一个组合。`degraded` 轨迹保存用于诊断，但永远不计为 `executable`。

## 当前运行时闭环

核心运行时按同一条可执行链处理四个贡献点：

- `SE2SequenceGenerator` 对 footprint 净空、可行朝向宽度、相邻朝向集合重叠、ESDF 修正距离和定位/控制/地图/形状不确定性进行归一化，并以加权 noisy-OR 生成连续 `risk_score`；Schmitt 滞回与 HIGH 邻域 guard band 将其稳定映射为 LOW/MEDIUM/HIGH。
- `SvsdfRuntime::EvaluateCandidate` 严格映射 LOW→R²、MEDIUM→SE(2)、HIGH→完整 SVSDF；HIGH 调用不允许以 `preserve_shape` 快速分支冒充完整 SVSDF。
- 拼接轨迹必须通过 footprint 净空、线/角速度、线/角加速度、跨 piece 连续性和请求起终点检查后，才能进入 validated incumbent。验证失败只对失败时刻所在的 guarded segment window 逐级升级；连续优化仍失败时才在同一局部窗口调用 Swept A*/Hybrid A* 搜索补丁。
- Hybrid A* fallback 与拓扑候选共享绝对 deadline。到时已有 validated incumbent 时仍返回该轨迹并保留 `timeout=true` 作为预算压力记录；没有 validated incumbent 时显式失败，离散中心线和部分搜索路径不会被发布为成功。

`GlobalReferencePoint` 及 `state_metrics.jsonl` 会同时记录三级标签、连续总分、五个分量风险及其几何原值；运行汇总另外记录三级求解调用时间、incumbent 更新/拒绝数和内部验证失败数。初始层级求解失败后的升级、前端 fallback、最终验证失败后的恢复分别由 `initial_level_upgrade_*`、`frontend_fallback_*` 和 `validation_recovery_*` 计数，三类事件不再混入同一个 recovery 指标。

每次运行时 validated incumbent 替换还会写入 `incumbent_events.jsonl`，包括单调时间戳、
候选编号、净空、平滑度、长度、持续时间、piece 数和三级区间数，可直接重建质量—时间
阶梯曲线。分析输出将该时钟明确命名为
`t_runtime_first_incumbent_ms`。旧结果只对最终返回轨迹执行一次实验层独立验证，因此无法
反推出“中间 incumbent 首次通过外部验证”的时刻；对于最终 `executable=true` 的运行，
分析器另以 `t_final_external_validation_ms` 记录外部验证完成后的整次运行完成时间，两者
不得混作同一个首次可行时间。发生局部瓶颈升级或搜索补丁并最终通过验证时，run 汇总按被替换的原始区间
弧长记录 `replanned_length_m`、实际补丁长度 `patch_length_m` 和
`trajectory_reuse_ratio=1-replanned_length/original_length`；没有真实局部替换时保持
`null`，不把整条最终轨迹伪装成“原轨迹”。

恢复汇总按 run-level 漏斗区分：触发最终验证恢复、至少一次局部恢复通过运行时验证、
该运行最终由规划器返回轨迹、最终返回轨迹通过实验层独立验证。每一级条件成功率均以
前一级为分母。`validation_recovery_success_count` 是局部运行时验证成功次数，不能与最终
`executable` 成功混用；历史字段 `recovery_success` 只表示搜索 patch 分支成功，在分析表中
单列为 `search_patch_success_run_count`，不再冒充所有局部恢复的成功率。轨迹复用率图只
显示实际记录该指标的运行，并标注有效样本数。

时间分解图只堆叠非重复的顶层计时。`t_full_svsdf_ms` 是
`t_strict_solve_ms` 的子集，`t_recovery_ms` 也与恢复中调用的求解器计时重叠，二者保留在
`planning_time_breakdown.csv` 作为嵌套诊断量，但不再次叠加到总高度。

## 依赖和构建

环境为 Ubuntu 20.04、ROS Noetic 和仓库原有 C++14/catkin 工具链。C++ 依赖由 `package.xml` 声明；Python 分析依赖见 `requirements.txt`，包括 pandas、NumPy、SciPy、Matplotlib、Pillow 和 PyYAML，不使用 seaborn。

```bash
cd ~/isweep_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
catkin_make run_tests
```

## 单次实验

```bash
roslaunch planner_experiments run_single_experiment.launch \
  tasks:=$(rospack find planner_experiments)/tasks/smoke.yaml \
  method:=current_full task_id:=task_001 \
  map_yaml:=$(rospack find planner_experiments)/maps/smoke_empty/map.yaml \
  robot_yaml:=$(rospack find planner_experiments)/config/robot_shapes/long_rectangle.yaml \
  output_dir:=/tmp/isweep_single_run run_id:=manual_001
```

## 批量实验

从 workspace 根目录执行：

```bash
python3 src/Implicit_Swept_Volume_Planner/planner_experiments/scripts/run_batch.py \
  --config src/Implicit_Swept_Volume_Planner/planner_experiments/config/experiment_default.yaml \
  --tasks src/Implicit_Swept_Volume_Planner/planner_experiments/tasks/smoke.yaml \
  --output results/main_experiment
```

旧的 `--tasks` 接口保留用于一张地图和一种形状。要在一个批次内运行当前定义的
全部 10 张唯一正式地图 × 4 种形状 × 6 个主方法，并在规划结束后自动做完整性检查、
统计、绘图和论文表格导出，执行一条命令：

```bash
python3 src/Implicit_Swept_Volume_Planner/planner_experiments/scripts/run_batch.py \
  --config src/Implicit_Swept_Volume_Planner/planner_experiments/config/experiment_default.yaml \
  --datasets src/Implicit_Swept_Volume_Planner/planner_experiments/datasets/all_maps_all_shapes.yaml \
  --output results/all_maps_all_shapes \
  --parallel-workers 1 \
  --postprocess
```

默认清单每个地图/形状生成 30 个经独立 footprint 起终点检查的任务，使用一个
planning seed，因此共运行 `10 × 4 × 30 × 6 = 7200` 个真实规划组合。先检查组合
而不运行规划可在同一命令末尾加 `--dry-run`。完整运行结束后的跨地图总表是：

```text
results/all_maps_all_shapes/<timestamp>/summary/cross_map_total_table.csv
```

其中每行是一组 `map_name × robot_shape × method_name`，包含样本数、五类成功率及
Wilson 95% 区间、总时间/首次可行时间/净空/长度/HIGH 比例的分布统计、strict 与
full-SVSDF 真实调用数，以及恢复统计。`svsdf_call_sources.csv` 将真实 full-SVSDF
调用进一步拆成初始分配、初始升级、前端 fallback、最终验证恢复、尾部修正和其他
来源；各来源之和必须等于 `full_svsdf_call_count`。`cross_map_method_summary.csv` 进一步对形状
聚合，`cross_map_executable_rate.csv` 是便于浏览的方法成功率宽表。

`datasets/all_maps_all_shapes.yaml` 的 10 张唯一地图包括 6 类版本化参数化地图
（窄门、L 转角、S 通道、连续旋转通道、双路线、随机障碍物）、`ditu`，以及
仓库原有 `map_clean_nav`、`maze`、`office`。`isweep_planner/maps/map.yaml` 与
`planner_experiments/maps/ditu/map.yaml` 的图像和地图参数完全相同，因此不会作为
第二个独立样本重复计数。快速验证矩阵调度时使用
`datasets/smoke_matrix.yaml`，不要把它的输出当作论文数据。

数据集清单同时支持两种来源：

```yaml
schema_version: 1
datasets:
  - id: hand_authored_key_case
    tasks: ../tasks/key_case.yaml
    metadata:
      scene_family: narrow_door
matrix:
  maps:
    - name: map_a
      yaml: package://planner_experiments/maps/map_a/map.yaml
  robots:
    - name: circle
      yaml: package://planner_experiments/config/robot_shapes/circle.yaml
  task_generation:
    num_tasks: 30
    seed: 1
    min_distance: 2.0
    yaw_mode: random
```

`datasets` 适合论文中人工定义、确保穿过瓶颈的关键任务；`matrix` 适合确定性生成
完整地图×形状笛卡尔积。两者可混合，所有生成任务副本均保存在批次目录并写入
manifest。参数化瓶颈实验的正式结论应优先使用经审查的人工任务，而不是仅依赖
随机起终点。

可选项为 `--methods`、`--maps`、`--robots`、`--task-ids`、`--seeds`、
`--resume`、`--overwrite`、`--parallel-workers`、`--dry-run` 和
`--postprocess`。默认单进程、单 OpenMP 线程；只有 `parallel_workers=1` 且线程数
为 1 时 manifest 才标记为正式计时。`--resume` 只跳过带原子 `complete` 标志的
run，并拒绝把不同实验配置、数据集清单或过滤条件续写进同一批次。

稳定 `run_id` 是方法、地图 hash、形状配置 hash、任务、seed 和完整实验配置 hash 的 SHA-256。Halton PRM 本身是确定性的，当前核心代码不消费随机 seed；seed 仍进入 run key 和 manifest，供随机地图及未来随机模块使用。

`planner_overrides` 可在实验 YAML 中覆盖真实 ROS 参数，例如 `topology/num_samples`，所有覆盖值随完整配置进入 manifest。正式实验应使用默认规划器配置；`experiment_smoke.yaml` 只用于快速链路测试。

## 地图生成

支持 `narrow_door`、`l_corner`、`s_channel`、`rotation_channel`、`dual_route` 和 `random_obstacles`：

```bash
rosrun planner_experiments generate_parametric_maps.py \
  --type narrow_door --resolution 0.05 --door-width 0.90 --seed 1 \
  --output-dir maps/generated/narrow_door_090
```

输出 `map.pgm`、`map.yaml` 和 `metadata.json`。metadata 包含生成器版本、尺寸、分辨率、几何参数、seed 与图像 SHA-256；同参数和 seed 生成同一栅格。

## 任务生成

```bash
rosrun planner_experiments generate_tasks.py \
  --map maps/generated/narrow_door_090/map.yaml \
  --robot config/robot_shapes/long_rectangle.yaml \
  --num-tasks 30 --seed 1 --min-distance 2.0 --yaw-mode random \
  --output tasks/narrow_door_long_rectangle.yaml
```

生成器独立做高密度 footprint 起终点占据检查。窄门、L 转角等关键场景建议使用手工任务；runner 还会拒绝越界、碰撞、非有限 yaw、相同起终点、重复 id 和重复姿态对。

`tasks/` 还提供与当前 `maps/` 中 9 张地图逐一同名的人工关键任务文件，每个文件
包含两个方向或两种穿越任务，默认绑定 `long_rectangle`。统一清单为
`datasets/all_hand_authored_map_tasks.yaml`，可以在一个 batch 中运行这些任务：

```bash
python3 planner_experiments/scripts/run_batch.py \
  --config planner_experiments/config/experiment_default.yaml \
  --datasets planner_experiments/datasets/all_hand_authored_map_tasks.yaml \
  --output results/all_hand_authored_map_tasks \
  --parallel-workers 1 \
  --postprocess
```

## 机器人形状和几何一致性

`config/robot_shapes/` 提供 `circle`（16 边形近似）、`long_rectangle`、`t_shape` 和 `l_shape`。每个文件同时给出 polygon、严格后端 mesh URI、安全余量及可选半径。正式规划前会比较 polygon 与 mesh 的 XY 投影面积、质心、双向轮廓距离、Hausdorff 距离和最大径向误差；不通过时记录 `CONFIGURATION_ERROR`，不调用规划器。

## 方法配置

`config/methods.yaml` 由同一个适配器解释：

- `current_full`：原有多候选、分层优化和完整恢复；
- `current_no_local_recovery`：纯局部恢复消融；仅关闭瓶颈邻域的层级升级与搜索 patch，保留 candidate switch、tail refinement 和前端 Hybrid A* fallback；
- `current_no_recovery`：保留风险分层，关闭局部升级、恢复和失败后候选切换；前端 Hybrid A* 兜底被显式保留；
- `full_r2`：合并全路径并只调用真实 `optimizeR2`；首个 LOW 候选求解或验证失败后整个运行立即失败，不尝试其他拓扑候选，不允许升级到 MEDIUM/HIGH，也不启用前端 Hybrid A* fallback；
- `full_strict`：保留与 `current_full` 相同的原始路径分段、候选调度与恢复开关，但把每段都强制标为 HIGH；每段调用 strict，且 `force_full_svsdf=true` 强制实际进入 LMBM 后端；
- `hybrid_astar`：使用现有 Hybrid A* 生成全局路径，之后使用同一轨迹输出和外部验证标准；
- `no_recovery`、`candidate_switch_only`、`local_upgrade_only`、`swept_astar_only`、`hybrid_astar_only`、`local_upgrade_and_candidate_switch`、`full_recovery`：同一运行时控制流上的恢复消融组合。

`strict_solver_call_count` 表示进入 strict 接口；`full_svsdf_call_count` 仅在实际执行 `optimize_traj_lmbm` 时增加，二者不可互换。

`full_strict` 与 `current_full` 现在共享相同的原始分段边界，差别是前者把所有段都送入 full-SVSDF，后者只对 HIGH 段调用。`current_full` 还可能因求解失败升级、最终验证恢复或尾部修正增加调用。正式分析应使用 `summary/svsdf_call_sources.csv` 判断调用来自哪个阶段，而不能只比较总数。

`current_full` 与 `current_no_local_recovery` 默认在首条通过 raw-grid footprint gate 的 incumbent 出现后，最多允许 4 次非尾部 full-SVSDF 调用；tail refinement 独立最多允许 1 次。实际调用数和被配额拒绝数记录在 `post_incumbent_svsdf_*` 与 `tail_refinement_svsdf_budget_reject_count` 字段中。

`full_strict` 的强制 HIGH 约束会在初始候选、前端 fallback、局部搜索生成的 `patch_segments` 和尾部修正等每个求解入口重新执行；strict patch 失败时禁止退回未计数的 `optimizeSE2`。完整性检查要求其 LOW/MEDIUM 调用均为零、每次 strict 调用都实际进入 full-SVSDF，且不得使用 preserve-shape bypass。

## 独立验证器

验证器不读取规划器 success。它用单调时间采样，并递归细分到同时满足最大时间、平移和 yaw 步长；yaw 先解缠。每个姿态旋转/平移 polygon，对边界和内部高密度采样，独立检查地图越界、原始占据栅格和 ESDF clearance。它还计算线速度/加速度、角速度/角加速度超限数，以及每个多项式 piece 边界的位置、速度、加速度、yaw 和 yaw-rate 跳变。

最终定义严格为：

```text
planner_returned && external_footprint_valid && dynamics_valid
&& boundary_continuous && !timeout && !degraded
```

`limits` 中的空值从真实规划器参数读取。若要求动力学验证但任一上限仍不可获得，则 `dynamics_valid=false`，不会假定一个默认值。

## 输出与空值策略

```text
<output>/<timestamp>/
├── manifest.json
├── runs.csv
├── runs.jsonl
├── state_metrics.jsonl
├── segment_metrics.jsonl
├── incumbent_events.jsonl
├── risk_ground_truth.jsonl
├── trajectories/<run_id>.csv
├── failures/<run_id>.json
├── logs/<run_id>.log
├── runs/<run_id>/
├── generated_tasks/<map>__<shape>.yaml
└── summary/
```

run 级字段覆盖环境/版本、请求、规划状态、阶段时间、风险、solver、恢复、轨迹质量和独立验证。状态/段日志可追溯到 `run_id` 与候选索引。轨迹列固定为 `t,x,y,yaw,vx,vy,ax,ay,yaw_rate,yaw_acceleration`，角度为弧度、距离为米、时间字段为毫秒。

分析后生成的 `summary/run_index.csv` 和 `summary/run_index.md` 是精简的人类可读
索引：每行直接列出地图、形状、方法、任务、状态、`run_id`、结果/轨迹/日志路径，
并包含可复制的 RViz 重放命令。

无法由现有接口真实观测的数值写 JSON `null`，CSV 写空单元格；不会写字符串 `nan`、估计值、随机值或固定占位数。已有真实的全局调用计数不会强行分摊到无法对应的单段字段。

## 结果检查、分析和论文表格

```bash
python3 src/Implicit_Swept_Volume_Planner/planner_experiments/scripts/check_results.py results/main_experiment
python3 src/Implicit_Swept_Volume_Planner/planner_experiments/scripts/analyze_results.py results/main_experiment
python3 src/Implicit_Swept_Volume_Planner/planner_experiments/scripts/export_paper_tables.py results/main_experiment
```

检查器生成 `result_integrity_report.md`。分析器只读取 `runs.jsonl`，生成方法/地图/形状、时间、恢复、风险和失败汇总，成功率使用 Wilson 95% 区间，连续指标使用确定 seed 的 bootstrap 95% 区间。十张图各自输出 PDF/PNG；没有真实 width 或风险 ground truth 数据时生成明确的 “No real data” 图，而不是合成曲线。`analysis_provenance.json` 绑定同一 manifest 和 runs 文件。

多地图批次另外输出 `cross_map_total_table.csv`、
`cross_map_method_summary.csv`、`cross_map_executable_rate.csv`，以及带 Wilson
误差条和各组样本数的 `success_rate_by_map_method.pdf/.png`。`--postprocess`
按“完整性检查成功 → 分析 → Markdown/LaTeX 导出”的顺序自动完成这些步骤；检查
失败时保留所有原始结果并返回非零状态，不会继续产生可能误导的汇总。

## 恢复和风险专项配置

恢复模块在 `methods.yaml` 逐项开关；日志区分尝试、成功和真实 Swept A* 展开数。Hybrid A* 当前类没有公开展开节点计数，因此该字段为 `null`。

`config/experiment_risk_ground_truth.yaml` 开启风险真值专项模式。规划器选定并发布
validated incumbent 后，对其每个真实风险段分别运行 LOW/R²、MEDIUM/SE(2) 与
强制完整 SVSDF，随后由实验层对三条段轨迹独立检查 footprint、动力学、piece
连续性和段端点。只有三个求解器都已尝试、完整 SVSDF 后端确实执行且成功时，
`label_available=true`。`l2_required` 表示 R² 不可执行而 SE(2) 可执行，
`l3_required` 表示 SE(2) 不可执行而 SVSDF 可执行；
`minimum_required_level` 另记录三者中的最低可执行层。三级求解的调用和
时间单独写入 `risk_ground_truth.jsonl` 与 `t_risk_ground_truth_ms`，不会污染主方法的
solver 调用计数或首次可行时间。专项运行的总墙钟时间包含标签生成，不应用于主方法
规划时延比较。真值专项的 strict 轨迹保留完整 SVSDF 原始输出，不启用通用
严格求解中的保形回滚。

```bash
python3 planner_experiments/scripts/run_batch.py \
  --config planner_experiments/config/experiment_risk_ground_truth.yaml \
  --datasets planner_experiments/datasets/all_maps_all_shapes.yaml \
  --output results/risk_ground_truth \
  --methods current_full --parallel-workers 1 --postprocess
```

分析器从真实三级标签生成 `risk_ground_truth_summary.csv`、L2/L3 风险十分位事件率、
混淆矩阵和 PR 曲线；若完整三级标签不存在则明确输出 No-data 图。`incumbent_events.jsonl` 同时
生成每个 run 的事件汇总和净空—时间阶梯图。

`precision_sweep` 给出推荐扫描轴，但当前 batch CLI 不自动做笛卡尔积；应生成相应分辨率地图，并用不同完整配置文件运行，配置 hash 会阻止误分组。

## RViz 已保存结果重放

```bash
roslaunch planner_experiments visualize_experiment.launch \
  result_directory:=results/main_experiment run_id:=<run_id>
```

重放会根据该 run 在 `manifest.json` 中记录的地图 hash/name 和原始任务文件自动定位
ROS map YAML，发布 `/planner_experiments/replay_map`，并同时发布保存的轨迹、风险状态、
safe-yaw 箭头和外部碰撞点，不重新规划。RViz 中 `Occupancy Map` 图层显示障碍物；
最终轨迹为青色、LOW 绿色、HIGH 红色、碰撞点黑色。如果实验结果被移动到另一台机器，
需要保持地图 package 可由 `rospack` 找到，或保持 manifest 中记录的地图路径有效。

当前保存格式尚未包含失败候选路径和恢复补丁几何，因此这两类过程仍不能完整重放。

## 常见错误与已知限制

- `pandas/scipy` 缺失：安装 `requirements.txt` 或相应 Ubuntu Python 包后再分析；规划和结果检查不依赖它们。
- geometry mismatch：检查 mesh 单位、原点、朝向及 polygon 顶点顺序，不要放宽验证器来迁就某个方法。
- `full_strict` 未见完整调用：检查 run 中 `full_svsdf_call_count`；正式配置必须保持 `force_full_svsdf: true`。
- VTK 导入目标/PCL 可选 I/O 警告是当前基线环境已有的非致命警告。
- 当前只读风险参考只覆盖最终选中候选；所有失败候选的状态/段级日志、L-BFGS/LMBM 迭代数、Hybrid A* 展开数、路径修正次数和恢复补丁长度仍没有可靠接口，保持为空。
- `trajectory_validator_node` 不是批处理入口；独立验证器已作为 C++ 库由 runner 同步执行。

## 可复现性

manifest 保存完整配置、配置 hash、Git commit/dirty、地图 metadata/hash、机器人几何、方法、任务数、seed、线程/并行状态、主机和构建信息。正式计时固定 `parallel_workers=1` 和 `fixed_cpu_threads=1`。每个 run 先写临时文件再原子 rename，最后才创建 `complete`；中断后使用 `--resume` 可安全继续。
