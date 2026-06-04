# TurtleBot3 移动机器人导航技术报告

> **ROS2 Humble + Gazebo Classic + Nav2 完整导航栈实现**

---

## 1. 项目概述

本项目基于 **ROS2 Humble** 和 **Nav2** 导航框架，在 **Gazebo Classic** 仿真环境中实现 **TurtleBot3 Burger** 移动机器人的自主导航功能。系统集成 **slam_toolbox** 进行实时 SLAM 建图，通过 **AMCL** 实现蒙特卡洛定位，最终由 Nav2 行为树驱动的全局/局部路径规划器完成目标点导航。

| 组件 | 技术选型 |
|------|----------|
| ROS 发行版 | ROS2 Humble Hawksbill |
| 仿真引擎 | Gazebo Classic 11 |
| 机器人平台 | TurtleBot3 Burger |
| SLAM | slam_toolbox (online_async) |
| 定位 | AMCL (Adaptive Monte Carlo Localization) |
| 全局规划器 | SmacPlannerHybrid |
| 局部控制器 | DWB (Dynamic Window Approach) |
| 行为树 | Nav2 Default BT |

---

## 2. 系统架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │  LiDAR   │  │   IMU    │  │  Odometry│  │  Diff Drive      │   │
│  │ (360°)  │  │ (accel/  │  │ (wheel   │  │  Controller      │   │
│  │         │  │  gyro)   │  │ encoder) │  │                  │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘   │
└───────┼──────────────┼──────────────┼─────────────────┼────────────┘
        │              │              │                 │
        ▼              ▼              ▼                 ▼
  /scan           /imu          /odom            /cmd_vel
        │              │              │                 ▲
        │              ▼              │                 │
        │    ┌──────────────────┐     │                 │
        └───►│  slam_toolbox    │     │                 │
             │  (online_async)  │     │                 │
             │                  │     │                 │
             └────────┬─────────┘     │                 │
                      │               │                 │
                  /map ▼              ▼                 │
             ┌──────────────────────────────────────┐   │
             │           Map Server                 │   │
             │  ┌─────────────┐  ┌───────────────┐  │   │
             │  │  Static Map │  │  Costmap      │  │   │
             │  │  (pgm/yaml) │  │  Layers       │  │   │
             │  └─────────────┘  └───────┬───────┘  │   │
             └───────────────────────────┼──────────┘   │
                                         │              │
                                         ▼              │
             ┌──────────────────────────────────────┐   │
             │    AMCL Localization                 │   │
             │  (Particle Filter)                   │   │
             │  ┌────────────────────────────┐      │   │
             │  │  /scan + /odom + /map      │      │   │
             │  │       → /tf (map→odom)    │      │   │
             │  └────────────────────────────┘      │   │
             └───────────────┬──────────────────────┘   │
                             │                          │
                             ▼                          │
             ┌──────────────────────────────────────┐   │
             │        Nav2 Behavior Tree             │   │
             │  ┌─────────────────────────────┐      │   │
             │  │ ComputePathToPose (Global)  │      │   │
             │  │         │                   │      │   │
             │  │    FollowPath               │      │   │
             │  │  ┌───────────────┐          │      │   │
             │  │  │ DWB Controller│──────────┼──────┘   │
             │  │  └───────────────┘          │          │
             │  │  On Failure: Recovery      │          │
             │  │  ┌──────┬──────┬──────┐    │          │
             │  │  │ Spin │Backup│ Wait │    │          │
             │  │  └──────┴──────┴──────┘    │          │
             │  └─────────────────────────────┘          │
             └──────────────────────────────────────────┘
```

### 2.2 组件说明

| 组件 | 功能描述 |
|------|----------|
| **Gazebo Simulation** | 提供物理仿真环境，包含 TurtleBot3 机器人模型、传感器插件（LiDAR, IMU）和差分驱动控制器 |
| **LiDAR (LDS-01)** | 360° 激光雷达，发布 `/scan` 话题 (LaserScan)，用于障碍物检测和 SLAM 扫描匹配 |
| **IMU** | 惯性测量单元，发布 `/imu` 话题，提供角速度和线加速度数据，辅助里程计融合 |
| **Odometry** | 轮式里程计，发布 `/odom` 话题和 `odom→base_footprint` TF 变换 |
| **slam_toolbox** | 基于 scan-to-map 匹配和位姿图优化的在线异步 SLAM，输出 `/map` 话题 |
| **Map Server** | 加载/保存 OccupancyGrid 地图，为 AMCL 和 Costmap 提供静态地图层 |
| **AMCL** | 自适应蒙特卡洛定位，粒子滤波器融合 `/scan`, `/odom`, `/map` 估计 `map→odom` 变换 |
| **Nav2 Planner** | 全局路径规划器 SmacPlannerHybrid，基于 Hybrid-A* 搜索生成从起点到目标的全局路径 |
| **Nav2 Controller** | 局部控制器 DWB，根据全局路径和局部代价地图计算速度指令 `/cmd_vel` |
| **Behavior Tree** | 协调规划、控制和恢复行为的状态机，处理导航失败时的自动恢复 |

---

## 3. TF 坐标变换树

### 3.1 TF Tree 结构

```
                         map
                          │
                  (AMCL publishes)
                          │
                          ▼
                         odom
                          │
                  (odometry publishes)
                          │
                          ▼
                   base_footprint
                          │
                  (robot_state_publisher)
                          │
                          ▼
                      base_link
              ┌───────────┼───────────┐
              │           │           │
              ▼           ▼           ▼
         base_scan   wheel_left   wheel_right
                      _link        _link
                          │
                          ▼
                       imu_link
```

### 3.2 坐标系说明

| 坐标系 | 发布者 | 作用 |
|--------|--------|------|
| **map** | AMCL | 全局世界坐标系，固定原点，用于全局定位和路径规划 |
| **odom** | 里程计/EKF | 里程计参考系，相对于启动位置，连续但存在累积漂移 |
| **base_footprint** | 里程计/robot_state_publisher | 机器人在地面的投影点，用于代价地图碰撞检测 |
| **base_link** | robot_state_publisher | 机器人本体中心坐标系，所有传感器和部件的参考原点 |
| **base_scan** | robot_state_publisher | LiDAR 传感器的安装位置，用于 scan→base_link 坐标变换 |
| **imu_link** | robot_state_publisher | IMU 传感器的安装位置 |
| **wheel_{left,right}_link** | robot_state_publisher | 左右驱动轮的旋转关节坐标系 |

> **关键关系**: `map → odom → base_footprint → base_link` 构成了完整的全局→局部→本体的 TF 链。AMCL 负责校正 `map→odom` 漂移，而 `odom→base_footprint` 由原始里程计维持高频更新。

---

## 4. Nav2 行为树分析

### 4.1 默认行为树结构

Nav2 采用行为树 (Behavior Tree) 而非传统的状态机来编排导航逻辑，提供更高的模块化和可组合性。

```
[ComputePathToPose]
        │
     [SUCCESS]
        │
        ▼
   [FollowPath]
        │
   ┌────┴────┐
   │         │
[SUCCESS] [FAILURE]
   │         │
   ▼         ▼
[Goal    [Recovery Subtree]
Reached]    │
       ┌────┼────┐
       │    │    │
      [Spin][Backup][Wait]
```

### 4.2 行为节点说明

| 节点 | 类型 | 功能 |
|------|------|------|
| **ComputePathToPose** | Action | 调用全局规划器 (SmacPlannerHybrid) 从当前位置到目标点生成全局路径 `nav_msgs/Path` |
| **FollowPath** | Action | 调用局部控制器 (DWB) 沿全局路径生成速度指令，实时避障 |
| **Spin** | Action | 原地旋转 360° 后重试，适用于机器人被完全包围时的脱困 |
| **Backup** | Action | 后退一定距离后重试，适用于前方有临时障碍物的情况 |
| **Wait** | Action | 等待一段时间（如 5 秒）后重试，适用于动态障碍物穿过的场景 |

### 4.3 代价地图层

| 层名称 | 类型 | 描述 |
|--------|------|------|
| **static_layer** | StaticLayer | 从 SLAM 地图加载的静态障碍物（墙壁、家具等），标记为 LETHAL (254) |
| **obstacle_layer** | ObstacleLayer | 来自 LiDAR `/scan` 的实时动态障碍物，持续更新 |
| **inflation_layer** | InflationLayer | 在障碍物周围扩展膨胀区域，膨胀半径由 `inflation_radius` 参数控制 |

> 代价地图采用 `[0, 255]` 代价值：`0` = 自由空间，`254` = 致命障碍，`255` = 未知区域。膨胀层在障碍物周围创建渐变代价衰减。

---

## 5. SLAM 流程

### 5.1 slam_toolbox 工作模式

本项目使用 `slam_toolbox` 的 **online_async** 模式——非实时异步 SLAM，适合无需高频地图更新的场景，计算压力更低。

**SLAM 流水线**:

```
1. 数据采集
   ┌──────────────────────────────────────┐
   │ /scan (LaserScan) → Laser 回调       │
   │ /odom (Odometry)  → 里程计位姿       │
   └──────────────────┬───────────────────┘
                      ▼
2. 扫描匹配 (Scan-to-Map)
   ┌──────────────────────────────────────┐
   │ 当前扫描 ←→ 已有子图                 │
   │ 使用 ICP / Correlative Scan Matching │
   │ 输出: 相对位姿变换 ΔT                 │
   └──────────────────┬───────────────────┘
                      ▼
3. 位姿图优化 (Pose Graph Optimization)
   ┌──────────────────────────────────────┐
   │ 节点: 每个关键帧的机器人位姿          │
   │ 边:   里程计约束 + 扫描匹配约束       │
   │ 求解: SPA (Sparse Pose Adjustment)   │
   │ 工具: g2o / Ceres Solver             │
   └──────────────────┬───────────────────┘
                      ▼
4. 地图更新
   ┌──────────────────────────────────────┐
   │ 合并优化后的子图 → OccupancyGrid     │
   │ 发布 /map 话题                       │
   └──────────────────────────────────────┘
```

### 5.2 地图保存与加载

```bash
# 保存地图 (在 SLAM 运行期间)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 输出文件:
#   my_map.pgm   — 栅格地图图像 (PGM 格式)
#   my_map.yaml  — 地图元数据 (分辨率, 原点, 阈值)

# 加载地图 (导航模式)
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/path/to/my_map.yaml
```

---

## 6. 路径规划

### 6.1 全局规划器: SmacPlannerHybrid

**SmacPlannerHybrid** 是 Nav2 的默认全局规划器，基于 **Hybrid-A\*** 搜索算法，适用于非完整约束的轮式机器人。

| 特性 | 说明 |
|------|------|
| 搜索算法 | Hybrid-A* (A* + 连续状态空间) |
| 运动模型 | Dubins / Reeds-Shepp 曲线 |
| 启发函数 | Obstacle Heuristic + Dijkstra Heuristic |
| 下采样因子 | `downsample_costmap: 1` (无下采样) |
| 角度量化 | `angle_quantization_bins: 72` (5° 分辨率) |
| 容差 | `tolerance: 0.25` m |

**工作原理**: 在 2D 代价地图上使用 A* 搜索，但每个节点不仅记录 `(x, y)` 位置，还记录朝向角 `θ`。使用 Dubins/Reeds-Shepp 曲线连接相邻节点，确保生成的路径满足机器人的最小转弯半径约束。通过障碍物启发函数引导搜索方向，大幅减少搜索空间。

```
传统 A*:      节点 = (x, y)
Hybrid-A*:    节点 = (x, y, θ) + 连续运动学连接
```

### 6.2 局部控制器: DWB

**DWB (Dynamic Window Approach)** 在速度空间 `(v, ω)` 中采样，评估每条轨迹的代价，选择最优速度指令。

**代价函数组成**:

| 评价器 (Critic) | 权重 | 功能 |
|-----------------|------|------|
| **PathAlign** | 1.0 | 轨迹与全局路径的对齐程度 |
| **PathDist** | 32.0 | 轨迹终点到路径的距离 |
| **GoalAlign** | 24.0 | 接近目标时的朝向对齐 |
| **GoalDist** | 24.0 | 轨迹终点到目标点的距离 |
| **ObstacleScale** | 1.0 | 与最近障碍物的距离惩罚 |

```
DWB 循环 (每 ~50ms):
1. 采样速度对 (v ∈ [v_min, v_max], ω ∈ [ω_min, ω_max])
2. 对每对速度，前向模拟轨迹 (sim_time)
3. 计算每条轨迹的总代价 = Σ (critic_i × weight_i)
4. 选择最小代价轨迹对应的速度
5. 发布 /cmd_vel (Twist)
```

### 6.3 代价地图配置

```yaml
# 全局代价地图 (规划用)
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.105
      resolution: 0.05
      width: 200         # 40m × 40m 范围
      height: 200
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

# 局部代价地图 (控制用)
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      robot_radius: 0.105
      resolution: 0.05
      rolling_window: true
      width: 60          # 3m × 3m 滑动窗口
      height: 60
      plugins: ["obstacle_layer", "inflation_layer"]
```

---

## 7. 参数调优指南

### 7.1 核心参数速查表

| 参数路径 | 推荐值 | 说明 |
|----------|--------|------|
| `planner_server.ros__parameters.tolerance` | 0.25 | 目标点到达容差 (m)，越小越精确但可能无法收敛 |
| `planner_server.ros__parameters.use_astar` | True | 启用 A* 搜索，False 则使用 Dijkstra |
| `controller_server.ros__parameters.max_vel_x` | 0.26 | 最大线速度 (m/s)，受 TurtleBot3 硬件限制 |
| `controller_server.ros__parameters.min_vel_x` | 0.0 | 最小线速度，设为 0 允许原地停止 |
| `controller_server.ros__parameters.max_vel_theta` | 1.0 | 最大角速度 (rad/s) |
| `controller_server.ros__parameters.min_vel_theta` | 0.2 | 最小角速度，避免机器人无法完成小幅转向 |
| `controller_server.ros__parameters.acc_lim_x` | 2.5 | 线加速度限制 (m/s²) |
| `controller_server.ros__parameters.acc_lim_theta` | 3.2 | 角加速度限制 (rad/s²) |
| `local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius` | 0.35 | 膨胀半径 (m)，增大可提升安全性 |
| `global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius` | 0.35 | 全局膨胀半径 |
| `amcl.ros__parameters.min_particles` | 500 | 最小粒子数 |
| `amcl.ros__parameters.max_particles` | 2000 | 最大粒子数 |
| `amcl.ros__parameters.update_min_d` | 0.1 | 最小平移距离触发的滤波器更新 |
| `amcl.ros__parameters.update_min_a` | 0.2 | 最小旋转角度触发的滤波器更新 |

### 7.2 调优策略

```
max_vel_x 调优:
  ├── 值过大 → 机器人撞墙 / 震荡
  ├── 值过小 → 导航缓慢
  └── 建议: 从 0.15 开始逐步上调

inflation_radius 调优:
  ├── 值过大 → 狭窄通道无法通过
  ├── 值过小 → 贴近障碍物行驶
  └── 建议: robot_radius + 0.1 ~ 0.2 (TurtleBot3: ~0.25-0.35)

tolerance 调优:
  ├── 值过大 → 目标点误差大
  ├── 值过小 → 原地调整无法收敛
  └── 建议: 0.15 ~ 0.3
```

---

## 8. 实验指标模板

### 8.1 SLAM 建图质量

| 测试场景 | 实际面积 (m²) | 建图面积 (m²) | 覆盖率 (%) | 平均定位误差 (m) | 建图耗时 (s) |
|----------|:-----------:|:-----------:|:--------:|:---------------:|:----------:|
| 空旷房间 |             |             |          |                 |            |
| 多障碍物 |             |             |          |                 |            |
| 长走廊   |             |             |          |                 |            |
| 复杂环境 |             |             |          |                 |            |

### 8.2 导航成功率

| 测试场景 | 总试验次数 | 成功次数 | 成功率 (%) | 恢复触发次数 | 碰撞次数 |
|----------|:--------:|:------:|:--------:|:----------:|:------:|
| 静态障碍物 |          |        |          |            |        |
| 动态障碍物 |          |        |          |            |        |
| 狭窄通道   |          |        |          |            |        |
| 多点巡航   |          |        |          |            |        |

### 8.3 导航效率

| 测试场景 | 欧氏距离 (m) | 实际路径长度 (m) | 路径/欧氏比 | 导航时间 (s) | 平均速度 (m/s) |
|----------|:----------:|:--------------:|:---------:|:----------:|:------------:|
| 直线导航 |            |                |           |            |              |
| L 型转弯 |            |                |           |            |              |
| U 型掉头 |            |                |           |            |              |
| 多弯道   |            |                |           |            |              |

---

## 9. 常见问题与解决

### 9.1 机器人不移动 (无 /cmd_vel 输出)

**现象**: 目标已下发，`/cmd_vel` 话题无数据或速度为零。

**排查步骤**:
```bash
# 1. 检查控制器状态
ros2 topic echo /controller_server/result

# 2. 检查是否有规划路径
ros2 topic echo /plan --once

# 3. 检查 TF 树完整性
ros2 run tf2_tools view_frames

# 4. 确认行为树未卡在某个节点
ros2 topic echo /behavior_tree_log
```

**解决方案**: 确认 `map→odom→base_link` TF 链完整，AMCL 已收敛（粒子云集中）。若路径规划失败，检查目标点是否在代价地图的自由空间内。

### 9.2 地图与机器人实际位置不对齐

**现象**: 机器人在地图中的显示位置与实际环境不匹配，激光扫描与地图墙壁有明显偏移。

**排查步骤**:
```bash
# 检查 AMCL 初始位姿
ros2 topic echo /initialpose

# 查看粒子分布 (Rviz2 中勾选 PoseArray)
ros2 topic echo /particle_cloud --no-arr

# 对比 /scan 与 /map 叠加效果
```

**解决方案**:
1. 在 Rviz2 中使用 "2D Pose Estimate" 工具手动给定准确的初始位姿
2. 增大 `min_particles` / `max_particles` 参数
3. 减小 `update_min_d` 和 `update_min_a` 以提高滤波器更新频率
4. 检查激光雷达与里程计的时延是否一致

### 9.3 路径规划失败

**现象**: `ComputePathToPose` 返回失败，终端输出 `[planner_server]: Plan failed`。

**排查步骤**:
```bash
# 查看代价地图，确认目标点未被标记为障碍物
ros2 run nav2_util costmap_info

# 检查规划器日志
ros2 run rqt_console rqt_console
```

**解决方案**:
1. **增大 tolerance**: 将 `tolerance` 从 0.1 调整为 0.25~0.5，降低到达精度要求
2. **检查 costmap**: 确保 `allow_unknown` 参数允许探索未知区域，或目标点落在已知区域内
3. **增大膨胀半径**: 适当减小 `inflation_radius` 以通过狭窄通道
4. **切换规划器**: 对于简单环境，可临时使用 `NavFnPlanner` 替代 `SmacPlannerHybrid`

### 9.4 TF 变换错误 (Lookup Would Require Extrapolation)

**现象**: 终端频繁输出 TF 外推错误，机器人模型在 Rviz2 中闪烁或错位。

```
[ERROR] [tf2]: Lookup would require extrapolation into the past.
```

**排查步骤**:
```bash
# 检查 TF 发布频率
ros2 run tf2_tools tf2_monitor

# 查看所有 TF 发布者
ros2 run tf2_ros tf2_echo map base_link

# 确认 URDF 中的 joint 和 link 定义正确
```

**解决方案**:
1. 在 `robot_state_publisher` 和 `joint_state_publisher` 启动文件中增加发布频率 (`publish_frequency: 50.0`)
2. 确保各节点使用 **simulation time** (`use_sim_time:=true`)，并检查 `/clock` 话题正常发布
3. 检查 URDF 文件中的 `<joint>` 定义，特别是 `wheel_left_link` 和 `wheel_right_link` 的旋转轴方向
4. 在 launch 文件中添加 `static_transform_publisher` 补充缺失的静态 TF

---

## 附录: 完整 Launch 命令

```bash
# 1. 启动 Gazebo 仿真环境
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 2. 启动 SLAM (建图模式)
ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=True
# 或使用 slam_toolbox
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True

# 3. 启动导航 (已知地图 + AMCL)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=/path/to/map.yaml

# 4. 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 5. 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

> **文档版本**: v1.0 | **日期**: 2026-05-23 | **作者**: Zexu
