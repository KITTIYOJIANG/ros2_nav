# ROS2/Nav2 控制算法案例复盘

## 1. 复盘背景

本案例的目标是跑通一个基于 ROS2 Humble、Nav2 和 TurtleBot3 的移动机器人导航 demo，并把运行过程整理成可复盘的工程材料。表面上看，这个任务只是“启动导航、截图、记录节点和话题”，但从控制算法角度看，它实际覆盖了移动机器人闭环控制的完整链路：环境地图提供全局约束，AMCL 根据激光雷达和里程计估计机器人在地图中的位姿，全局规划器根据起点和目标点生成路径，局部控制器再把路径转化成 `/cmd_vel` 速度指令。最终的验证证据包括节点、话题、TF、地图、全局路径、局部路径、速度命令和 rosbag 记录，因此这个项目可以作为一个 ROS2 控制算法案例来复盘。

本次运行环境是 WSL Ubuntu 22.04 + ROS2 Humble。验收目录为 `demo/nav2_official_tb3_20260604_115048/`。其中 `nav2_runtime_evidence.png` 是总览截图，`runtime_tf_tree.png` 和 `tf_tree.png` 是 TF 证据，`map_turtlebot3_world.yaml/.pgm/.png` 是地图证据，`nav2_goal_bag/` 是运行录包。最终 Nav2 lifecycle 节点进入 `active [3]`，`/navigate_to_pose` 目标被接受，rosbag 捕获了 1169 条消息，持续约 29.4 秒，包含 `/cmd_vel`、`/cmd_vel_nav`、`/odom`、`/scan`、`/tf`、`/tf_static`、`/map`、`/plan`、`/local_plan` 和 `/amcl_pose`。

## 2. 问题复盘

本案例最关键的问题不是 Nav2 本身无法启动，而是 Gazebo TurtleBot3 仿真链路中 `/spawn_entity` 服务没有正常暴露。官方 TurtleBot3 Gazebo launch 和 Nav2 的 `tb3_simulation_launch.py` 都尝试过，日志中可以看到 Gazebo factory plugin 已经加载，但 spawn 服务迟迟不可用，导致机器人实体无法按标准流程生成。如果继续卡在 Gazebo 侧，验收需要的 Nav2 节点、TF、路径和速度输出都会被阻塞。

第二个问题是 TF 链路。Nav2 对 TF 非常敏感，尤其需要 `map -> odom -> base_footprint/base_link` 这条链稳定存在。初期日志里出现过 `Invalid frame ID "map"` 和 `Lookup would require extrapolation into the past` 这类错误，说明 Nav2 costmap 或 planner 在查询 `base_link` 到 `map` 的变换时，时间戳和 frame 尚未完全对齐。AMCL 没有初始位姿时不会稳定发布 `map -> odom`，里程计和 base frame 不存在时也无法形成可用的机器人位姿。

第三个问题是验收证据要完整。仅截图一个 RViz 或仅列出节点都不够，因为验收标准明确要求节点、话题、TF、地图和运行截图。控制算法项目尤其需要证明“闭环真的通了”：有 goal、有 plan、有 local_plan、有 cmd_vel，才能说明导航栈不仅启动，而且完成了从目标到速度指令的控制计算。

## 3. 建模复盘

这个工程可以抽象成一个差速移动机器人的二维导航控制问题。机器人状态可以简化为 `x, y, theta`，其中 `x, y` 表示机器人在地图坐标系下的位置，`theta` 表示朝向。控制输入是线速度 `v` 和角速度 `omega`，在 ROS2 中对应 `geometry_msgs/Twist`，通过 `/cmd_vel` 发布。差速模型的基本形式是：

```text
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = omega
```

在 Nav2 中，这个模型不是由我们手写控制器直接求解，而是分布在多个模块里协同完成。`map_server` 提供静态栅格地图，AMCL 融合 `/scan`、`/odom` 和地图信息估计 `map -> odom`，从而把局部里程计坐标和全局地图坐标联系起来。全局规划器读取 costmap 和目标点，输出 `/plan`，也就是从当前位置到目标的全局路径。局部控制器读取全局路径、局部 costmap、机器人当前位姿和障碍物信息，输出 `/cmd_vel`。

本次证据中的 runtime TF 树非常重要：`map -> odom -> base_footprint -> base_link` 表示全局定位、局部里程计和机器人本体之间的坐标关系已经串起来。`base_link -> base_scan` 则说明激光雷达数据可以变换到机器人坐标系，进而用于 costmap 障碍物更新。没有这条 TF 链，规划器即使能生成路径，控制器也无法判断机器人相对路径的位置偏差，更无法生成可靠速度。

## 4. 控制算法理解

Nav2 的控制不是一个单一 PID，而是一套“规划 + 跟踪 + 恢复”的行为树控制框架。`/navigate_to_pose` 接收到目标后，BT Navigator 会先调用 `ComputePathToPose`，让 planner server 生成全局路径；然后调用 `FollowPath`，让 controller server 沿路径发布速度。若中途失败，行为树还可以触发 Spin、Backup、Wait 等恢复行为。

从控制角度看，全局规划解决“应该走哪条路”，局部控制解决“下一时刻怎么动”。局部控制器通常会在一组候选速度中采样，预测机器人短时间内的运动轨迹，然后根据多个代价项打分，例如距离全局路径的偏差、距离目标的偏差、朝向误差、障碍物距离等。代价最低的速度会被发布为 `/cmd_vel`。因此 `/cmd_vel` 是算法闭环的最终输出，`/plan` 和 `/local_plan` 是中间决策证据。

这个项目里还实现了一个 `waypoint_follower.py`，它使用 Nav2 Simple Commander API 封装多目标点导航逻辑。它的工程意义在于把底层 action 调用变成更高层的任务流程：加载 waypoint、等待 Nav2 active、发送 goal、读取 feedback、失败时清 costmap 并重试。面试或汇报时可以把它讲成“任务层调度”，而 Nav2 planner/controller 是“运动规划与控制层”。

## 5. 调参与排障过程

本次调参和排障的优先级不是一上来改 controller 参数，而是先保证系统可观测。第一步检查 ROS2 环境和包是否存在，例如 `nav2_bringup`、`slam_toolbox`、`turtlebot3_gazebo`、`tf2_tools` 和 `rviz2`。第二步记录 `ros2 node list`、`ros2 topic list`、`ros2 action list`，确认 Nav2 的基础服务已经启动。第三步检查 lifecycle 状态，确保 `map_server`、`amcl`、`controller_server`、`planner_server`、`bt_navigator` 等节点进入 `active [3]`。

由于 Gazebo spawn 服务不可用，本次采用了工程上的绕行策略：仍然使用官方 Nav2 launch、地图、AMCL、planner、controller 和 BT navigator，但用一个轻量 fake TurtleBot3 过程补齐 `/odom`、`/scan` 和 `odom -> base_footprint` TF。这个做法的目的不是替代真实仿真，而是在当前 WSL 限制下验证 Nav2 控制链路本身。换句话说，问题被拆成两层：Gazebo 实体生成是仿真问题，Nav2 是否能完成定位、规划和速度输出是导航控制问题。验收任务关注后者，所以先保证控制链路可运行。

在参数理解上，需要重点关注四类参数。第一类是速度约束，例如最大线速度、最大角速度和加速度限制，值太大会抖动或冲过目标，值太小会导致导航慢甚至无法脱困。第二类是 goal tolerance，容差过小会让机器人在目标附近反复微调，容差过大则到点精度下降。第三类是 inflation radius，膨胀半径过大可能让窄通道被 costmap 堵死，过小则容易贴近障碍物。第四类是 AMCL 粒子数和更新阈值，粒子太少会定位不稳，太多会增加 CPU 压力。

## 6. 结果与证据

最终结果表明，Nav2 栈在该环境中完成了从初始位姿、目标接收、全局规划、局部规划到速度输出的闭环验证。`/navigate_to_pose` goal 被接受后，`controller_server` 开始计算控制量，日志中出现 `Received a goal, begin computing control effort`，随后多次 `Passing new path to controller`，最后出现 `Reached the goal!`。这说明 planner 与 controller 之间的数据链路是通的。

rosbag 记录是最有说服力的结果证据。它捕获 1169 条消息，其中 `/cmd_vel` 和 `/cmd_vel_nav` 各 364 条，`/local_plan` 364 条，`/plan` 7 条，`/tf` 39 条，`/odom` 22 条。这组数据说明控制器不是只发布了一次速度，而是在导航过程中持续输出速度指令；局部路径也在持续更新；TF 和 odom 为控制计算提供了位姿基础。地图文件显示分辨率为 0.05 m/cell，runtime TF 图显示 `map -> odom -> base_footprint -> base_link` 链路存在。

从工程复盘角度看，这次案例的价值在于完成了一次“导航系统集成调试”的闭环：遇到 Gazebo 生成失败时，没有停在单点问题上，而是拆分验证目标，先证明 Nav2 控制链路可运行；同时把所有证据落盘，形成可复查材料。后续如果继续完善，优先方向是修复 Gazebo factory spawn 服务，让真实 TurtleBot3 实体接入 `/cmd_vel`；其次是补充更多地图和目标点测试，统计成功率、平均路径长度、平均速度和到点误差；最后可以把 DWB 或 Regulated Pure Pursuit 的参数做对比实验，形成更扎实的控制算法调参报告。

## 7. 我的学习收获

这个项目教会我的第一点是：ROS2 导航不是单个节点，而是一组节点通过话题、action、service 和 TF 组成的系统。排障时不能只盯启动日志，要沿着数据流检查：map 是否有，initial pose 是否发了，AMCL 是否发布 `map -> odom`，planner 是否输出 `/plan`，controller 是否输出 `/cmd_vel`。第二点是：TF 是移动机器人系统的骨架。只要 TF 断了，定位、规划、控制都会变成无源之水。第三点是：控制算法落地时，调参一定要结合现象。机器人贴障碍物，优先看 inflation 和 footprint；目标附近抖动，优先看 tolerance 和角速度；完全不动，优先查 lifecycle、TF、costmap 和 `/cmd_vel`。

因此，这个案例可以总结为：在 ROS2/Nav2 中，一个可运行的控制算法系统不只是“有算法名”，而是要有状态估计、坐标变换、规划输出、局部控制输出和可复现证据。最终录包和截图证明本项目完成了最小闭环，也为后续真实仿真和参数实验打下基础。
