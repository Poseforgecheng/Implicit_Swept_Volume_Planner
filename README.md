# iSweep: Risk-Guided Three-Level Trajectory Planning

[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)](https://isocpp.org/)

面向任意形状地面机器人的连续几何风险驱动轨迹规划器，根据局部几何难度分配 **R² → SE(2) → 完整 SVSDF** 优化。验证失败只升级或替换瓶颈邻域；只有通过统一验证的轨迹才能更新当前最优可行解（validated incumbent）。

本仓库提供 ROS/RViz 交互规划、离线实验与独立验证、IR-SIM Ackermann 闭环对比以及被动实验记录。RViz 仅负责显示，不是碰撞真值或车辆仿真器。底盘/雷达驱动、实车定位栈和 TEB 不包含在本仓库。

## 本次更新

- 同步五因素风险、滞回和 HIGH 邻域保护带，默认演示启用 LOW/MEDIUM/HIGH 三级，而不是旧两级模式。
- 同步风险边界共享切线、碰撞感知拐角预平滑和拼接后平滑；修改后的轨迹仍须重新验证。
- footprint、动力学、piece 连续性及请求起终点 gate 先于 incumbent 替换和严格调用配额激活。
- `full_r2` 失败后禁止升级；`full_strict` 的初始、局部恢复与尾部修正段均强制 HIGH，并进入完整 SVSDF 后端。
- 整条参考轨迹风险着色；几何风险标签不与强制求解层级混淆。
- 补齐 incumbent 事件流、三级独立风险真值、恢复漏斗、真实替换复用率及分来源 SVSDF 计数。
- 加入四方法顺序仿真、对比绘图和回归测试。失败候选只用于诊断，不作为成功轨迹。

当前核心包含历史实验之后的平滑和部署修改，**新运行不能标记为论文原始实验记录**。

## 1. 方法与边界

```text
Map / ESDF / footprint / mesh / start / goal
  -> Halton roadmap + repeated Dijkstra
  -> shortcut + ESDF correction + approximate deduplication
  -> five-factor continuous risk + hysteresis / guards
  -> LOW: R² | MEDIUM: SE(2) | HIGH: full SVSDF
  -> stitch + footprint / dynamics / continuity / endpoint gates
  -> validated incumbent + report, or explicit failure
       failure -> guarded window W -> upgrade / local search patch -> revalidate
       front-end shortage -> Hybrid A* fallback under the same scheduling budget
```

五因素为 footprint 有效净空、可行朝向宽度、朝向集合连续性、局部修正代价和不确定性侵蚀。风险分数用于分配求解精度，**不是校准后的碰撞概率**。

当前导航输入为二维占据栅格，状态为 `(x,y,yaw)`。严格后端采用网格形状计算扫掠几何，但不等于已经实现一般三维环境导航。平移/旋转限制验证以及 Ackermann 搜索/命令 gate，不代表优化后端完整施加了自行车模型和转向角约束。

时间预算用于调度和提前返回，后端/验证可能造成预算越界，**不宣称硬实时截止保证**。最终可执行结果、首次已验证轨迹时间和指定时间内的可用率必须分别统计。

## 2. 仓库结构

```text
isweep_planner/       # 规划核心、消息、地图、mesh、RViz
planner_experiments/  # 离线批量实验、独立验证、JSONL/CSV、后处理
irsim_bridge/         # IR-SIM 状态/碰撞真值、四方法对比、绘图
navigation_adapter/  # 地图、定位、起终点、重规划请求的 ROS 适配
planner_bridge/      # 隔离输出话题的 Ackermann 命令 gate
tools/               # 被动记录器及测试
```

## 3. 安装与编译

基准环境：Ubuntu 20.04、ROS Noetic、C++14、Python 3.8。IR-SIM 单独使用 Python ≥3.10，不替换 ROS 系统 Python。

```bash
sudo apt-get update
sudo apt-get install build-essential cmake libeigen3-dev libpcl-dev libyaml-cpp-dev libssl-dev \
  python3-catkin-pkg python3-empy python3-nose python3-numpy python3-pil python3-yaml \
  python3-pandas python3-scipy python3-matplotlib \
  ros-noetic-map-server ros-noetic-rviz ros-noetic-message-generation \
  ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-pcl-conversions \
  ros-noetic-roscpp ros-noetic-rospy ros-noetic-roslib ros-noetic-rosbag \
  ros-noetic-sensor-msgs ros-noetic-std-msgs ros-noetic-std-srvs \
  ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-msgs ros-noetic-visualization-msgs

mkdir -p ~/isweep_ws/src
cd ~/isweep_ws/src
git clone https://github.com/Poseforgecheng/Implicit_Swept_Volume_Planner.git
cd ~/isweep_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
source devel/setup.bash
```

`isweep_planner/lib/` 包含预编译依赖，`lmbm.so` 为 Linux x86-64。虽然源码有 ARM64 编译分支，但 ARM/不同 ABI 上必须重编匹配的第三方库，不能直接保证现有二进制可用。内存不足请用 `-j1`。

## 4. 三级 RViz 演示

```bash
source ~/isweep_ws/devel/setup.bash
roslaunch isweep_planner demo.launch \
  enable_three_level_planner:=true planner_method:=current_full \
  use_start_goal_params:=false
```

先用 **2D Pose Estimate** 选起点，再用 **2D Nav Goal** 选终点，箭头方向决定 yaw。发布版 `demo.launch` / `planner.launch` 默认启用三级，且默认 `run_local_planner:=false`，不会启动真实底盘。`enable_three_level_planner:=false` 可关闭部署叠加配置，回到基础 YAML。

更换地图：

```bash
roslaunch isweep_planner demo.launch \
  map_file:=$(rospack find isweep_planner)/maps/serpentine_corridor.yaml \
  planner_method:=current_full
```

地图 YAML 的 image、resolution、origin 必须正确。整个 footprint 应位于自由空间，不仅是中心点。若已有外部 `/map`，设置 `use_map_server:=false`。

### 配置入口

| 文件 | 用途 |
|---|---|
| `isweep_planner/config/planner.yaml` | 矩形部署基础配置，0.45 × 0.36 m footprint 与 box mesh |
| `isweep_planner/config/planner_v3_ackermann.yaml` | 默认叠加三级、8 s 调度预算、拐角/拼接平滑、验证和调用配额 |
| `isweep_planner/config/planner_benchmark.yaml` | 离线实验基础配置，机器人由实验 YAML 指定，与交互平滑参数分离 |
| `planner_experiments/config/experiment_default.yaml` | 六方法批量实验，默认20 s配置预算及单线程计时记录 |
| `planner_experiments/config/experiment_risk_ground_truth.yaml` | 各标签段三级独立求解与验证 |

修改机器人时必须同步 footprint 和 mesh。实验层检查面积、质心与轮廓距离。`planner_config:=...` 可指定基础 YAML；若用 `planner_benchmark.yaml` 且不希望叠加部署参数，请同时设置 `enable_three_level_planner:=false`，benchmark 自身仍使用三级风险。

### 方法切换

| `planner_method` | 含义 |
|---|---|
| `current_full` | Proposed，风险驱动三级优化与局部恢复 |
| `full_r2` | R2-only，LOW 失败即失败，禁止升级、恢复和前端 fallback |
| `full_strict` | Full-SVSDF，保留分段与调度，每个优化段强制 HIGH |
| `hybrid_astar` | Hybrid A* 搜索及统一最终验证 |

以上四个名称用于 ROS/IR-SIM。离线 runner 还支持 `current_no_local_recovery`、`current_no_recovery` 等消融，见 [methods.yaml](planner_experiments/config/methods.yaml)，不能将任意消融名传给 ROS 节点。

## 5. ROS 话题与显示

| 话题 | 含义 |
|---|---|
| `/map` | 占据栅格输入 |
| `/initialpose`、`/move_base_simple/goal` | 请求起终点 |
| `/isweep_planner/trajectory` | 已验证规划轨迹 |
| `/isweep_planner/trajectory_astar` | 可能未验证或不完整的诊断候选 |
| `/isweep_planner/risk_aware_global_reference` | 整条参考轨迹、风险标签、五因素及几何原值 |
| `/isweep_planner/markers` | footprint 和风险显示 |
| `/isweep_planner/planning_time` | 规划耗时 |
| `/isweep_planner/planning_stats` | 数组式遥测，字段顺序见记录器，不猜测下标 |
| `/isweep_planner/planner_method` | 当前方法 |

`risk_reference_low` 为绿色、`risk_reference_medium` 为黄/橙色、`risk_reference_high` 为红色，它们是同一完整参考轨迹的风险区间。`full_strict` 的求解层级全部为 HIGH，但几何风险显示仍可有三种颜色。

## 6. 离线实验与后处理

从仓库根目录执行，使用新的输出目录：

```bash
cd ~/isweep_ws/src/Implicit_Swept_Volume_Planner
source ~/isweep_ws/devel/setup.bash

# 链路测试，不作为正式计时
python3 planner_experiments/scripts/run_batch.py \
  --config planner_experiments/config/experiment_smoke.yaml \
  --tasks planner_experiments/tasks/smoke.yaml \
  --output results/smoke --postprocess

# 只检查矩阵，不运行规划
python3 planner_experiments/scripts/run_batch.py \
  --config planner_experiments/config/experiment_default.yaml \
  --datasets planner_experiments/datasets/all_maps_all_shapes.yaml \
  --robots t_shape circle long_rectangle \
  --output results/main_comparison --dry-run
```

确认矩阵后，将 `--dry-run` 改为 `--parallel-workers 1 --postprocess` 才会启动批量实验。清单也包含 L 形，三形状实验需保留上述过滤；实际组合以 manifest 为准。

独立三级风险真值：

```bash
python3 planner_experiments/scripts/run_batch.py \
  --config planner_experiments/config/experiment_risk_ground_truth.yaml \
  --datasets planner_experiments/datasets/all_maps_all_shapes.yaml \
  --robots t_shape --parallel-workers 1 \
  --output results/risk_truth_t_shape --postprocess
```

已有 JSONL 可重新统计，不必重新运行规划：

```bash
python3 planner_experiments/scripts/check_results.py RESULTS_DIRECTORY
python3 planner_experiments/scripts/analyze_results.py RESULTS_DIRECTORY
python3 planner_experiments/scripts/export_paper_tables.py RESULTS_DIRECTORY
```

失败/超时保留在分母中。区分初始升级、前端 fallback 和验证后恢复。`incumbent_events.jsonl` 记录已验证替换事件；复用率只计真实接受的局部替换，其余不填0。`t_full_svsdf_ms` 嵌套于 strict 时间，不能重复相加。详见 [实验说明](planner_experiments/README.md)。

## 7. IR-SIM 闭环与四方法对比

```bash
cd ~/isweep_ws/src/Implicit_Swept_Volume_Planner
./irsim_bridge/scripts/setup_irsim_env.sh
source ~/isweep_ws/devel/setup.bash
roslaunch irsim_bridge isweep_v3_irsim_rviz.launch \
  map_file:=$(rospack find isweep_planner)/maps/office.yaml \
  planner_method:=current_full
```

安装脚本固定 IR-SIM 2.10.2，使用包内隔离虚拟环境。仿真输出 `/irsim/cmd_vel`，不接真实 `/cmd_vel`。IR-SIM 提供车辆状态、碰撞与到达判断，RViz 仅显示。

顺序运行同地图与起终点的四种方法（yaw 单位 rad）：

```bash
rosrun irsim_bridge run_planner_comparison.py \
  --map-file "$(rospack find isweep_planner)/maps/serpentine_corridor.yaml" \
  --start 2 2 1.57079632679 --goal 28 2 -1.57079632679 \
  --methods current_full full_r2 full_strict hybrid_astar \
  --seed 7 --timeout 120 --output results/irsim_comparison/case01

rosrun irsim_bridge plot_planner_comparison.py results/irsim_comparison/case01
```

`--timeout` 是单方法仿真执行/观测限制，不是离线主实验20 s规划预算。相同起终点不保证四个方法均成功；未验证候选与超时结果在图中明确标注。详见 [IR-SIM 说明](irsim_bridge/README.md)。

## 8. 实车数据记录

由自己的导航栈提供地图、定位、里程计和规划话题后，启动只读记录器：

```bash
python3 tools/scripts/record_planning_evidence.py \
  --output results/physical/session_001 --mode isweep_v3
```

记录新目标之后首次/最新参考、实际轨迹、风险、规划统计、跟踪误差和 rosbag，拒绝覆盖已有会话。`--mode` 仅是记录标签，不切换算法。超时或 Ctrl+C **不会停止底盘**，急停必须由外部控制链负责。错误基于定位流，不是外部真值精度。详见 [记录工具](tools/README.md)；[实车协议](isweep_planner/docs/real_robot_v3_protocol.md)中的硬件启动器属于外部导航工作区。

## 9. 测试

```bash
cd ~/isweep_ws
catkin_make run_tests -j2
catkin_test_results build/test_results
cd src/Implicit_Swept_Volume_Planner
python3 -m unittest discover -s tools/test -v
```

回归覆盖强制方法分支、风险单调升级、raw-grid footprint 检查、拼接切线、拐角平滑、诊断搜索链、复用率、地图/任务生成、IR-SIM 运动转换和记录器时间戳。单元测试不代替所有场景的端到端实验或实车测试。

本次发布的实际检查及未覆盖范围见 [RELEASE_VALIDATION.md](RELEASE_VALIDATION.md)。后处理所需 pandas/SciPy 等依赖必须安装在运行分析脚本的同一 Python 环境中。

## License and acknowledgements

各 `package.xml` 保留原许可证声明；仓库目前没有顶层 `LICENSE` 文本，本次更新未替第三方组件重新授权。libigl、nanoflann、LMBM 及预编译依赖保留原有许可，特别是 copyleft/restricted 组件不能被当成统一 MIT 授权。

扫掠体积几何与粗到细规划参考了 ESV / Implicit-SVSDF 相关研究。感谢上游作者及 ROS、IR-SIM、几何优化依赖的维护者；保留原有代码署名及第三方文件，不将相关算法全部宣称为原创。
