# 三级规划 Ackermann 实车实验流程

本文为外部硬件导航工作区的集成协议，不是本仓库的一键实车启动器。`start_navigation_headless_test.sh`、`sentry_nav`、TEB、底盘/雷达/定位驱动需由用户自己的部署栈提供。本文路径为示例；本仓库提供的只读记录器位于 `tools/scripts/`，可直接用 ROS Python 运行。请先完成隔离仿真，再按真实机器人标定验证。

## 1. 实验目的

本实验验证当前 LOW–MEDIUM–HIGH 三级规划器生成的轨迹能否在真实 MID360 定位和 Ackermann 底盘上执行。首轮采用 `isweep_v3_teb`，将规划可执行性与自研局部控制器性能分离。实验不重复仿真中的大规模成功率或风险真值统计。

## 2. 启动前 gate

1. 实车完整外轮廓必须被 `planner.yaml` 中的 0.45 m × 0.36 m footprint 和 `robot_box_45x36x50.obj` 包含。
2. Ackermann 最小转弯半径按 1.0 m 配置；选点应使中心线转弯半径不小于 1.2 m。
3. 急停可用，现场人员位于车体运动范围外。
4. 初次启动先架空驱动轮，核对正速度、左右转向和停止方向。
5. 只在已完成定位对齐的先验地图覆盖区域运行；初轮不加入临时动态障碍。

任一条件不满足时，不发送 RViz 导航目标。

## 3. 编译

消息定义和三级核心已更新，首次使用必须重新编译：

```bash
cd ~/nav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

连接硬件前可先检查脚本解析结果；该命令不清理已有 ROS 进程，也不启动雷达、串口或底盘：

```bash
cd ~/robot_navigation
NAV_CONFIG_ONLY=1 ./start_navigation_headless_test.sh isweep_v3_teb
```

## 4. 低速试运行

```bash
cd ~/robot_navigation
BASE_LINEAR_X_SCALE=0.5 \
ACKERMANN_MODE=true \
ACKERMANN_MIN_TURN_RADIUS=1.0 \
./start_navigation_headless_test.sh isweep_v3_teb
```

该命令直接连接底盘并启用 `/cmd_vel`。`BASE_LINEAR_X_SCALE=0.5` 仅用于安全调试；正式记录恢复为 1.0，否则 CSV 中的 `/cmd_vel` 不等于底盘最终施加速度。

在 RViz 中先发送 `2D Pose Estimate`，等待定位日志出现 `Initialize successfully!!!!!!`，并确认激光与先验地图稳定重合 10–20 s，再启动记录器和发送 `2D Nav Goal`。

## 5. 数据记录

在线记录器：

```bash
source /opt/ros/noetic/setup.bash
source ~/nav_ws/devel/setup.bash

rosrun sentry_nav experiment_recorder.py \
  --mode isweep_v3_teb \
  --timeout 120 \
  --position-tolerance 0.10 \
  --yaw-tolerance 0.20 \
  --dwell-time 1.0 \
  --output ~/experiment_data/real_robot_v3/runs.csv
```

每个任务使用独立 rosbag：

```bash
rosbag record \
  -O ~/experiment_data/real_robot_v3/scene_a_run01.bag \
  /tf /tf_static /initialpose /move_base_simple/goal \
  /localization /Odometry /scan /prior_map /cmd_vel \
  /isweep_planner/current_pose \
  /isweep_planner/trajectory /isweep_planner/trajectory_astar \
  /isweep_planner/risk_aware_global_reference \
  /isweep_planner/reference_risk /isweep_planner/reference_clearance \
  /isweep_planner/planning_time /isweep_planner/planning_stats \
  /isweep_planner/markers /isweep_teb_local_planner/status
```

同时使用固定机位录像，记录 run id、场景、起终点、速度缩放、是否人工急停及异常现象。

## 6. 场景顺序

1. 开阔区 2–3 m 直线：检查方向、定位与停车。
2. 5–8 m 宽 L 形路线：转弯半径至少 1.2 m，检查连续航向和跟踪误差。
3. 静态地图中的直穿门洞：门宽先选 0.8–0.9 m，起终点距门洞至少 1.5 m，不在门内急转。

先完成每个场景 1 次试运行；稳定后每个场景记录 5 次，共 15 次。避免原地旋转、U 形掉头、窄门后立即 90° 转向和未映射动态障碍。

## 7. 成功判据

- 无接触、无人工急停、无定位丢失；
- 终点位置误差不超过 0.10 m；
- 终点航向误差不超过 0.20 rad；
- 上述条件持续至少 1 s；
- 有有效三级风险参考和通过统一 gate 的轨迹；
- 无持续超过记录器允许间隔的控制中断。

CSV 的 `minimum_lidar_range_m` 是传感器量程，不是 footprint 净空。论文若报告真实车体净空，应使用 rosbag 中的定位、静态栅格与相同 footprint 离线重算。

## 8. 论文可报告内容

建议报告 15 次运行的成功率、终点误差、导航时间、实际距离、跟踪误差均值/P95、三级风险段构成、首次 incumbent 时间和 full-SVSDF 调用。配图采用实车场景照片、规划/执行轨迹叠加以及速度—风险随时间变化。

结论限定为“当前三级规划在所测矩形 Ackermann 平台上的物理可执行性”，不将小规模实车结果外推为任意形状统计泛化。
