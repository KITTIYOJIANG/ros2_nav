# TurtleBot3 Burger URDF 分析文档

> 适用环境：ROS2 Humble + Gazebo (Ignition/Classic)
> 文档语言：中文（技术术语保留英文）
> 用途：机器人学作品集 / 学习笔记

---

## 1. TurtleBot3 Burger 概述

TurtleBot3 Burger 是一款小型、低成本的双轮差速驱动机器人，广泛应用于 ROS 教学与 SLAM/Navigation 研究。

### 物理参数

| 参数 | 数值 |
|------|------|
| 最大速度（平移） | 0.22 m/s |
| 最大速度（旋转） | 2.84 rad/s |
| **车轮半径** | **0.033 m (33 mm)** |
| **轮距 (wheelbase)** | **0.160 m (160 mm)** |
| 整机高度 | ~192 mm |
| 整机宽度 | ~138 mm |
| 重量 | ~1 kg |
| 载重 | ~15 kg |

### 传感器套件

| 传感器 | 型号 | 用途 |
|--------|------|------|
| **LDS 激光雷达** | LDS-01 / LDS-02 | 360° 二维激光扫描，SLAM 建图与障碍物检测 |
| **IMU 惯性测量单元** | 板载 IMU (Gyroscope + Accelerometer + Magnetometer) | 姿态估计、角速度测量 |
| **摄像头（可选）** | Raspberry Pi Camera Module v2 | 视觉任务、目标识别 |

---

## 2. URDF / Xacro 结构拆解

TurtleBot3 使用 **xacro** 宏语言编写 URDF 模型，主入口文件为 `turtlebot3_burger.urdf.xacro`，通过 `#include` 引入各子模块。

### 2.1 常用 Xacro 宏 (Macros)

以下宏定义在 `turtlebot3_burger.gazebo.xacro` 及通用宏文件中，用于复用几何 / 惯性 / 传感器定义。

#### inertial 宏族

```xml
<!-- 圆柱体惯性 -->
<xacro:macro name="cylinder_inertial" params="m r h">
  <inertial>
    <mass value="${m}" />
    <inertia ixx="${m*(3*r*r+h*h)/12}" iyy="${m*(3*r*r+h*h)/12}" 
             izz="${m*r*r/2}" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</xacro:macro>

<!-- 球体惯性 -->
<xacro:macro name="sphere_inertial" params="m r">
  ...
</xacro:macro>

<!-- 长方体惯性 -->
<xacro:macro name="box_inertial" params="m w d h">
  ...
</xacro:macro>
```

#### wheel 宏

```xml
<xacro:macro name="wheel" params="prefix x_reflect y_reflect">
  <link name="wheel_${prefix}_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial m="0.005" r="${wheel_radius}" h="${wheel_width}"/>
  </link>

  <joint name="wheel_${prefix}_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="wheel_${prefix}_link"/>
    <origin xyz="${x_reflect*wheel_separation/2} 0 -${wheel_radius}" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>
```

参数说明：
- `prefix`: `left` 或 `right`，区分左右轮
- `x_reflect`: `-1` (左轮) 或 `1` (右轮)，用于沿 X 轴镜像放置
- `y_reflect`: 通常为 `1`，用于沿 Y 轴镜像

#### sensor_lidar 宏

```xml
<xacro:macro name="sensor_lidar" params="prefix">
  <link name="base_scan">
    <visual>
      <geometry><cylinder radius="0.060" length="0.030"/></geometry>
    </visual>
    <xacro:cylinder_inertial m="0.105" r="0.06" h="0.018"/>
  </link>
  <joint name="base_scan_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="base_scan"/>
    <origin xyz="-0.052 0 0.111"/>
  </joint>
</xacro:macro>
```

---

### 2.2 所有 Link 一览

| Link 名称 | 用途 | 惯性宏 | 备注 |
|-----------|------|--------|------|
| `base_footprint` | 投影在地面的虚拟原点，无质量 | 无 | 所有里程计和定位的参考系起点 |
| `base_link` | 机器人本体的几何中心 | `cylinder_inertial` | 包含主体圆柱体 visual/collision |
| `base_scan` | 激光雷达安装位置 | `cylinder_inertial` | 通过 fixed joint 固定在 base_link 上方 |
| `wheel_left_link` | 左驱动轮 | `cylinder_inertial` | 通过 continuous joint 绕 Z 轴旋转 |
| `wheel_right_link` | 右驱动轮 | `cylinder_inertial` | 同上 |
| `caster_back_link` | 后置万向轮（从动） | `sphere_inertial` | 通过 fixed joint 固定，仅用于外观与摩擦 |
| `imu_link` | IMU 传感器安装位置 | 无（质量忽略） | 通过 fixed joint 固定，仅用于 tf 发布 |

---

### 2.3 所有 Joint 一览

| Joint 名称 | 类型 | 父 Link | 子 Link | 说明 |
|------------|------|---------|---------|------|
| `base_joint` | **fixed** | `base_footprint` | `base_link` | 固定偏移，将机器人抬离地面（Z 向平移 wheel_radius 高度） |
| `wheel_left_joint` | **continuous** | `base_link` | `wheel_left_link` | 左轮旋转关节，无角度限制，绕 Z 轴旋转 |
| `wheel_right_joint` | **continuous** | `base_link` | `wheel_right_link` | 右轮旋转关节 |
| `caster_back_joint` | **fixed** | `base_link` | `caster_back_link` | 后置万向球固定，不参与驱动 |
| `base_scan_joint` | **fixed** | `base_link` | `base_scan` | 激光雷达固定安装，偏移至顶部 |
| `imu_joint` | **fixed** | `base_link` | `imu_link` | IMU 固定安装 |

**Joint 类型说明：**
- `fixed`: 两个 link 之间无相对运动，仅定义空间偏移（静态变换）
- `continuous`: 无限旋转关节（无硬限位），适用于车轮，角度从 `-∞` 到 `+∞`

---

## 3. TF 树结构 (ASCII Art)

```
                    odom  (里程计坐标系，由 robot_localization / ekf_node 维护)
                     |
                     |  (动态 tf，随运动漂移)
                     |
               base_footprint  (机器人在地面的投影原点)
                     |
                     |  static tf: z = +wheel_radius (0.033m)
                     |
                  base_link  (机器人几何中心)
                  /    |    \
                 /     |     \
     static tf  /      |      \  static tf
               /  cont.|cont.  \
              /        |        \
   base_scan    wheel_left_link   wheel_right_link
  (LiDAR)         (左驱动轮)       (右驱动轮)
      |
      | static tf
      |
   imu_link  (IMU 传感器)
```

**节点说明：**

| 边 | 变换类型 | 发布者 | 说明 |
|----|---------|--------|------|
| `odom` → `base_footprint` | 动态 | `robot_localization` / `ekf_node` | 融合 wheel odometry 和 IMU 数据估计位姿 |
| `base_footprint` → `base_link` | static | `robot_state_publisher` | 固定 Z 偏移，高为 wheel_radius |
| `base_link` → `wheel_left_link` | 动态 (joint) | `robot_state_publisher` | 读取 `/joint_states` 中的轮子角度 |
| `base_link` → `wheel_right_link` | 动态 (joint) | `robot_state_publisher` | 同上 |
| `base_link` → `base_scan` | static | `robot_state_publisher` | LiDAR 安装位置固定 |
| `base_link` → `caster_back_link` | static | `robot_state_publisher` | 万向球固定 |
| `base_link` → `imu_link` | static | `robot_state_publisher` | IMU 安装位置固定 |

> 在实际 TurtleBot3 硬件上，`base_footprint` → `base_link` 的偏移为 `Z = +0.033`（车轮半径），而在仿真中此值保持一致，确保物理行为可迁移。

---

## 4. Gazebo 插件详解

TurtleBot3 Burger 的 `turtlebot3_burger.gazebo.xacro` 中定义了三个关键 Gazebo 插件，每个插件将 URDF 模型与 Gazebo 物理引擎和 ROS2 Topic 体系对接。

### 4.1 Differential Drive 差速驱动插件

```xml
<gazebo>
  <plugin name="turtlebot3_burger_diff_drive" 
          filename="libgazebo_ros_diff_drive.so">
    <!-- 里程计坐标系 -->
    <odometry_frame>odom</odometry_frame>

    <!-- 接收 /cmd_vel 中的 Twist 消息 -->
    <ros>
      <namespace>/</namespace>
      <argument>cmd_vel:=cmd_vel</argument>
    </ros>

    <!-- 轮子配置 -->
    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
    <wheel_separation>0.160</wheel_separation>  <!-- 轮距 -->
    <wheel_radius>0.033</wheel_radius>          <!-- 车轮半径 -->

    <!-- 发布参数 -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>false</publish_wheel_tf>
  </plugin>
</gazebo>
```

**功能说明：**
- **接收** `/cmd_vel` (geometry_msgs/Twist)：接收来自 Nav2 / teleop 的速度指令
- **发布** `/odom` (nav_msgs/Odometry)：根据轮子编码器仿真计算里程计数据
- **发布** `odom → base_footprint` 的 tf 变换
- **核心公式**（内部实现）：
  - 线速度 `v = (ω_left + ω_right) * wheel_radius / 2`
  - 角速度 `ω = (ω_right - ω_left) * wheel_radius / wheel_separation`

### 4.2 LiDAR 激光雷达插件 (Ray Sensor)

```xml
<gazebo reference="base_scan">
  <sensor name="lds_lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>            <!-- 每圈 360 个采样点 -->
          <resolution>1</resolution>         <!-- 1° 角分辨率 -->
          <min_angle>0.0</min_angle>
          <max_angle>6.28319</max_angle>     <!-- 2π 弧度 = 360° -->
        </horizontal>
      </scan>
      <range>
        <min>0.120</min>                     <!-- 最小探测距离 12cm -->
        <max>3.5</max>                       <!-- 最大探测距离 3.5m -->
        <resolution>0.015</resolution>       <!-- 距离分辨率 15mm -->
      </range>
    </ray>
    <plugin name="turtlebot3_burger_laserscan" 
            filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <argument>out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

**功能说明：**
- **类型**：Ray Sensor（光线投射传感器）
- **发布** `/scan` (sensor_msgs/LaserScan)：360° 激光扫描数据（距离数组 + 角度信息）
- **参数**：360 个采样点（1° 分辨率），范围为 0.12m ~ 3.5m，模拟真实 LDS-01 参数
- **消耗** SLAM Toolbox、Nav2 的 `costmap` 均依赖此话题

### 4.3 IMU 惯性测量单元插件

```xml
<gazebo reference="imu_link">
  <gravity>true</gravity>
  <sensor name="turtlebot3_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian">0.00015</noise></x>
        ...
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian">0.0005</noise></x>
        ...
      </linear_acceleration>
    </imu>
    <plugin name="turtlebot3_imu_plugin" 
            filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**功能说明：**
- **类型**：IMU (Inertial Measurement Unit) 传感器
- **发布** `/imu` (sensor_msgs/Imu)：线加速度 + 角速度 + 姿态四元数
- **噪声模型**：高斯噪声，模拟真实 IMU 的测量误差
- **用途**：`robot_localization` 将 IMU 数据与 wheel odometry 融合，提升里程计精度

---

## 5. 关键 ROS2 Topic 从 URDF 派生的发布关系

以下表格梳理了每个 topic 由哪个源头发布、以及哪些节点消费它们：

| Topic | 消息类型 | 发布者 | 消费者 | 说明 |
|-------|---------|--------|--------|------|
| `/tf` | tf2_msgs/TFMessage | `robot_state_publisher` + `gazebo_ros_diff_drive` | SLAM Toolbox, Nav2, Rviz2 | 所有 link 间的坐标变换 |
| `/tf_static` | tf2_msgs/TFMessage | `robot_state_publisher` | 同上 | fixed joint 的静态变换（仅发布一次） |
| `/joint_states` | sensor_msgs/JointState | `robot_state_publisher` (从 Gazebo 读) | `robot_state_publisher` 自身 / Rviz2 | 所有 joint 的位置、速度、力矩 |
| `/odom` | nav_msgs/Odometry | `gazebo_ros_diff_drive` 插件 | `robot_localization`, Nav2 (`amcl`) | 轮式里程计 |
| `/scan` | sensor_msgs/LaserScan | `gazebo_ros_ray_sensor` 插件 (LiDAR) | SLAM Toolbox, Nav2 (costmap) | 360° 激光扫描 |
| `/imu` | sensor_msgs/Imu | `gazebo_ros_imu_sensor` 插件 | `robot_localization`, Nav2 | 惯性测量数据 |
| `/cmd_vel` | geometry_msgs/Twist | Teleop / Nav2 Controller | `gazebo_ros_diff_drive` 插件 | 速度控制指令 |

**数据流示意图：**

```
 /cmd_vel (controller输出)
     │
     ▼
 [gazebo_ros_diff_drive] ──► 物理仿真 (Gazebo 引擎)
     │                          │
     ├──► /odom (nav_msgs/Odometry)
     ├──► tf: odom → base_footprint
     │
     ▼
 [gazebo_ros_ray_sensor] ──► /scan (sensor_msgs/LaserScan)
 [gazebo_ros_imu_sensor] ──► /imu  (sensor_msgs/Imu)
     │
     ▼
 [robot_state_publisher]  ◄── /joint_states
     │
     ├──► tf: base_footprint → base_link → ... (static + joint)
     └──► /tf_static (fixed transformations)
```

---

## 6. 如何修改 URDF 进行定制

### 6.1 修改车轮半径

在 `turtlebot3_burger.urdf.xacro` 中，找到 `<xacro:property name="wheel_radius" ...>` 并修改数值：

```xml
<!-- 原始值 -->
<xacro:property name="wheel_radius" value="0.033"/>

<!-- 改为更大的轮子（适用于越野场景） -->
<xacro:property name="wheel_radius" value="0.045"/>
```

**同步修改**：
- `turtlebot3_burger.gazebo.xacro` 中 Gazebo diff drive 插件的 `<wheel_radius>` 必须同步更新
- `base_joint` 的 Z 偏移量也依赖 `wheel_radius`，需确认宏中已使用变量引用而非硬编码

### 6.2 修改底盘尺寸

```xml
<!-- 增大机器人本体半径 -->
<xacro:property name="body_radius" value="0.080"/>    <!-- 原值 0.070 -->

<!-- 加宽轮距以提高稳定性 -->
<xacro:property name="wheel_separation" value="0.200"/> <!-- 原值 0.160 -->
```

> 若修改 `wheel_separation`，同样需要同步更新 Gazebo diff drive 插件中对应的 `<wheel_separation>` 标签。

### 6.3 添加摄像头 Link

```xml
<!-- 新增摄像头 link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.016 0.024 0.016"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="0.003"/>
    <inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<!-- 新增 fixed joint 将摄像头固定到 base_link -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child  link="camera_link"/>
  <origin xyz="0.05 0 0.08" rpy="0 0 0"/>
</joint>

<!-- Gazebo 摄像头插件 -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/</namespace>
        <argument>image_raw:=camera/image_raw</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

添加后将自动发布 `/camera/image_raw` (sensor_msgs/Image) 话题。

### 6.4 快速验证

修改后使用 `check_urdf` 工具验证语法：

```bash
# 将 xacro 编译为纯 URDF 并检查
xacro turtlebot3_burger.urdf.xacro | check_urdf

# 查看生成的 TF 树
xacro turtlebot3_burger.urdf.xacro > /tmp/tb3.urdf
urdf_to_graphviz /tmp/tb3.urdf
```

---

## 7. 仿真 URDF 与真实 TurtleBot3 的对比

| 维度 | 仿真 (Gazebo) | 真实 TurtleBot3 |
|------|--------------|-----------------|
| **URDF 结构** | 完全相同 — link 名称、joint 类型、坐标偏移与实物一致 | 同左 |
| **`wheel_radius` / `wheel_separation`** | 需与实物一致（均为 0.033m / 0.160m） | 由机械设计决定，不可更改 |
| **diff drive 插件** | `libgazebo_ros_diff_drive.so` 提供里程计仿真 | 无此插件，里程计由 OpenCR 固件直接从编码器计算并通过 `/odom` 发布 |
| **LiDAR** | Gazebo Ray Sensor 插件仿真 360° 扫描 | LDS-01 实体的 `hlds_laser_publisher` 节点读取串口数据并发布 `/scan` |
| **IMU** | Gazebo IMU Sensor 插件仿真加速度与角速度 | OpenCR 板载 IMU 通过 `turtlebot3_imu` 节点发布 `/imu` |
| **`robot_state_publisher`** | 读取 `/joint_states` 发布 tf（仿真中 joint_states 由 Gazebo 提供） | 完全相同的节点，但 `/joint_states` 来自 OpenCR 编码器读数 |
| **TF 树** | `odom → base_footprint → base_link → ...` 完全一致 | 完全一致 |
| **新增传感器** | 只需在 URDF 中添加 link + Gazebo 插件即可运行 | 需额外的硬件接线和驱动节点 |

**关键结论：** TurtleBot3 的仿真 URDF 与真实机器人共享完全相同的 **运动学模型** 和 **传感器坐标系结构**。这意味着：

1. 在仿真中开发调试的 Nav2 参数（如 `inflation_radius`、`max_vel_x` 等）可直接迁移到实物
2. 仿真中训练的 SLAM 算法配置适用于实物部署
3. 仿真中添加新传感器的 URDF 结构为真正的硬件集成提供了参考拓扑

唯一不能从仿真迁移的是 Gazebo 插件本身 —— 真实机器人使用硬件驱动节点代替仿真插件，但发布的话题名称和消息类型保持一致，确保上层算法（Nav2、SLAM Toolbox）无需任何修改即可切换运行环境。

---

## 参考资料

- [TurtleBot3 官方 eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS2 URDF 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [turtlebot3_description GitHub](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_description)

---

*文档编写日期：2026-05-23*
*作者：Zexu，用于机器人学作品集*
