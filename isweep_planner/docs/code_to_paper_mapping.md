# iSweep 三级部署版代码—论文映射

本文档对应论文《有限时间下连续几何风险驱动的任意形状机器人三级轨迹规划》。`nav_ws` 同时保留兼容配置和当前三级配置；只有启用 `planner_v3_ackermann.yaml` 的 `isweep_v3`/`isweep_v3_teb` 才作为论文当前方法的实车部署入口。

本独立发布仓库的 `demo.launch` / `planner.launch` 默认加载三级叠加配置，且默认关闭局部速度输出。文中 `isweep_v3` / `isweep_v3_teb` 模式名属于外部实车导航工作区，不是本仓库自带的底盘启动器。批量实验单独使用 `planner_benchmark.yaml`。

## 1. 运行配置

| 启动模式 | 全局规划 | 局部跟踪 | 风险层级 | 用途 |
| --- | --- | --- | --- | --- |
| `isweep` | iSweep | 风险感知跟踪器 | LOW/HIGH 兼容配置 | 历史回退 |
| `isweep_teb` | iSweep | TEB | LOW/HIGH 兼容配置 | 历史回退 |
| `isweep_v3` | 当前 iSweep | 风险感知跟踪器 | LOW/MEDIUM/HIGH | 完整部署链测试 |
| `isweep_v3_teb` | 当前 iSweep | TEB | LOW/MEDIUM/HIGH | 首选实车可执行性验证 |

三级实车配置位于 `config/planner_v3_ackermann.yaml`。该文件叠加在 `planner.yaml` 之后，因此继承实车矩形 footprint、mesh、速度限制与话题，只覆盖三级风险、有限时间、严格查询和统一验证参数。

## 2. 连续几何风险

主要实现：

- `include/isweep_planner/core/common.h`
- `include/isweep_planner/optimization/midend/se2_sequence_generator.h`
- `src/optimization/midend/se2_sequence_generator.cpp`

`RiskSample` 保留论文中的五类原始量和归一化风险分量：

- footprint 净空及不确定性侵蚀后的有效净空；
- 可行 yaw 集合宽度；
- 相邻 yaw 集合连续性；
- 路径位置/航向修正代价；
- 定位、控制、地图及形状误差形成的不确定性侵蚀。

五个分量通过 noisy-OR 融合为 `score`，再经双阈值、Schmitt 滞回和 HIGH 邻域保护带生成 LOW、MEDIUM、HIGH 连续风险段。兼容模式通过 `risk/enable_three_level=false` 禁用 MEDIUM；三级模式必须显式设置为 `true`。

## 3. 三级求解与统一验证

主要实现：

- `src/framework/svsdf_runtime_evaluate.cpp`
- `src/optimization/backend/trajectory_optimizer.cpp`
- `src/optimization/backend/se2_svsdf_solver.cpp`
- `src/core/trajectory_footprint_validator.cpp`

求解映射为：

- LOW：轻量 `R^2` 位置优化；
- MEDIUM：显式 `SE(2)` 状态优化；
- HIGH：严格 `SE(2)` 并进入完整 SVSDF 后端。

层级升级保持 `R^2 -> SE(2) -> SVSDF` 单调关系。拼接结果必须通过 footprint、动力学、piece 边界和请求起终点 gate 后才能成为 incumbent。实车配置沿用正式实验的密集 footprint 采样与自适应时间/平移/yaw 细分规则。

## 4. 局部恢复与有限时间调度

主要实现：

- `src/framework/recovery_manager_bottleneck.cpp`
- `src/framework/recovery_manager_tail.cpp`
- `src/framework/svsdf_runtime_plan.cpp`

验证失败后仅升级瓶颈保护窗口；连续优化仍失败时，允许在同一窗口内尝试 strict patch、Swept A* 或 Hybrid A*。每次替换重新经过统一 gate。窗口外轨迹保持不变，并记录原轨迹长度、替换长度、补丁长度和复用率。

`RuntimeExperimentTelemetry` 区分初始层级升级、前端 fallback、最终验证恢复、candidate switch、tail refinement 和各来源 full-SVSDF 调用。`isweep_planner_node` 将这些字段追加发布到 `/isweep_planner/planning_stats`，供实车记录器写入 CSV。

## 5. ROS 输出

`/isweep_planner/risk_aware_global_reference` 中每个参考点包含：

- `x, y, yaw, s, segment_id`；
- `risk_level`：0/1/2 分别表示 LOW/MEDIUM/HIGH；
- `risk_score` 与五个风险分量；
- 原始/有效净空、可行 yaw 宽度与重叠度、修正距离；
- 风险相关建议速度。

`experiment_recorder.py` 进一步记录规划与执行指标、LOW/MEDIUM/HIGH 点数与段数、五因素极值、首次 incumbent 时间、三级求解调用、full-SVSDF 来源、恢复漏斗、轨迹复用率及计划—实车横向误差。

## 6. 论文结论边界

`isweep_v3_teb` 用于证明当前三级规划输出能够在真实定位和 Ackermann 底盘上被成熟局部控制器执行；它不验证自研局部跟踪器。`isweep_v3` 才覆盖自研跟踪链，但应在前者稳定后测试。

当前实车矩形模型只能作为单一矩形平台的可执行性证据，不能代替 T 形、圆形和长矩形批量实验，也不能单独证明任意形状泛化。临时动态障碍主要由局部控制层处理，不应据此宣称全局连续风险已在线重计算。
