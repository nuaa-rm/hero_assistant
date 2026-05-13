# RoboMaster 英雄小助手 (Hero Assistant)

基于 ROS2 的 RoboMaster 赛车自主导航与吊射辅助系统，主要面向英雄机器人。

## 功能概述

本项目提供以下核心功能：

1. **实时距离测算** — 持续计算并下发当前机器人与目标点（敌方基地装甲板）之间的距离信息，包含 x、y、z 三维坐标差值。目标点需在建完赛场地图后手动标注（可能存在人工标注误差），距离信息通过串口协议（0x0606）发送至下位机。

2. **一键标记传送锚点** — 比赛中可一键将当前机器人的位置与姿态记录为传送锚点。

3. **一键自主导航至传送锚点** — 标记锚点后，可一键触发自主导航，机器人将利用 Nav2 导航栈自主规划路径前往之前标记的传送锚点。

> **注意：** 功能 2 和 3（导航相关）此前主要在哨兵上使用，对精度要求不高且无方向约束，到达目标点的判断条件较宽松，尚未经过精细调校。

## 测试情况

| 项目 | 状态 |
|------|------|
| 已经在哨兵上全流程验证 | 已跑通，等待上车 |
| 3502 等有墙壁等强特征环境 | 定位精度 ±1cm，距离精度 ~1cm |
| 空旷赛场 | 精度略低，尚未完整测试 |
| 导航至传送锚点精度 | 粗估 10-20cm 误差，尚未专项调校 |

## 项目结构

```
hero_assistant/
├── robot_bring_up/              # 系统启动包（launch 文件、配置文件）
│   ├── launch/
│   │   ├── sentry.launch.py     # 哨兵主启动文件
│   │   ├── bringup_launch.py    # Nav2 + navigation_manager 启动
│   │   ├── serial.launch.py     # 串口通信启动
│   │   ├── localization_launch.py   # 定位（map_server）启动
│   │   ├── navigation_launch.py     # Nav2 导航栈启动
│   │   ├── pcd2pgm.launch.py        # PCD 转栅格地图
│   │   └── safeinit.launch.py
│   ├── config/
│   │   ├── sentry.yaml          # 全局参数配置
│   │   └── betterRvizConfig.rviz
│   └── maps/                    # 场地地图文件
│
├── robot_serial/                # 串口通信节点
│   ├── src/
│   │   ├── robot_serial_node.cpp
│   │   └── robot_serial.cpp     # 上位机 ↔ C板通信
│   └── include/serial_pro/      # 串口协议相关头文件
│
├── robot_navigation/            # 导航管理节点
│   └── src/
│       └── navigation_manager.cpp   # 距离计算 + 导航命令处理
│
├── rm_interfaces/               # 自定义 ROS2 接口
│   ├── msg/
│   │   ├── Receive/             # 下位机 → 上位机
│   │   │   ├── NavigationCommand.msg  # 导航命令（1=标记, 2=导航到锚点, 3=导航到目标）
│   │   │   ├── Robotp.msg
│   │   │   └── Robotstatus.msg
│   │   └── Transmit/            # 上位机 → 下位机
│   │       ├── DistanceInfo.msg # 距离信息（topic: /robot/distance_info, 协议 0x0606）
│   │       ├── SelfPosition.msg # 位置信息（给自瞄用）
│   │       └── Velocity.msg     # 底盘速度指令
│   └── srv/
│       └── CalculateDistance.srv
│
├── Point-LIO/                   # 激光雷达惯性里程计（定位与建图）
│   ├── config/pointlio.yaml
│   ├── src/
│   ├── launch/
│   └── PCD/                     # 保存的点云地图
│
├── robot_mapping/               # 地图工具
│   ├── merge_pcd/               # 多 PCD 合并
│   └── pcd2pgm/                 # PCD 转二维栅格地图
│
├── obstacle_segmentation_tc/    # 障碍物分割节点
│   └── 基于点云法向量识别障碍物，输出给 costmap
│
├── robot_bt_navigator/          # 行为树导航（自定义 BT 节点）
│   └── behavior_trees/          # 各种行为树 XML
│
├── robot_behaviors/             # 自定义 Nav2 行为插件
│   └── keep_away_from_obstacles # 避障行为
│
├── nav2_custom_costmap_plugin/  # 自定义 costmap 层（动态移除障碍物区域）
│
├── config_dynamic_modifier/     # 配置动态修改器（RViz 中标记隧道/移除区域）
│
├── teb_local_planner/           # TEB 局部路径规划器（自适应时间弹性带）
│
└── .gitignore
```

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                   上位机 (ROS2)                       │
│                                                     │
│  Livox 驱动 ──→ Point-LIO ──→ TF (map → base_link) │
│                   │                                  │
│                   ├──→ navigation_manager            │
│                   │     ├─ 距离计算 → /robot/distance_info │
│                   │     └─ 导航命令处理              │
│                   │                                  │
│  Nav2 (导航栈) ←─── navigation_manager              │
│  ├─ bt_navigator                                     │
│  ├─ controller_server (TEB)                         │
│  ├─ planner_server                                  │
│  ├─ behavior_server                                 │
│  └─ map_server                                      │
│                                                     │
│  robot_serial ←──→ 串口 ──→ C板（下位机）            │
│    ├─ 接收 NavigationCommand（标记/导航命令）         │
│    ├─ 发送 DistanceInfo（距离信息）                   │
│    └─ 发送 Velocity（速度指令）                       │
│                                                     │
│  obstacle_segmentation ──→ costmap                   │
│  config_dynamic_modifier ──→ 动态修改 costmap 参数    │
└─────────────────────────────────────────────────────┘
```

## 依赖环境

- **操作系统**：Ubuntu 22.04
- **ROS2 版本**：Humble（推荐）
- **关键依赖**：
  - Nav2 (Navigation2)
  - livox_ros_driver2
  - PCL (Point Cloud Library)
  - tf2 / tf2_ros

## 编译

```bash
# 进入工作空间
cd ~/hero_assistant_ws
mkdir -p src
git clone git@github.com:cmyhj/hero_assistant.git

# 编译全部包
colcon build

# source 环境
source install/setup.bash
```

## 使用方法

### 完整功能启动

```bash
# 启动哨兵主系统（包含定位、建图、障碍物分割、Nav2 导航、导航管理）
ros2 launch robot_bring_up sentry.launch.py
```

`sentry.launch.py` 中的布尔变量控制各子模块：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `if_rviz` | `True` | 是否启动 RViz2 |
| `if_sim` | `False` | 是否使用仿真时间（Gazebo） |
| `if_map` | `False` | 建图模式（为 `True` 时跳过障碍物分割和导航栈） |

### 串口通信单独启动

```bash
# 串口节点（支持自动重启）
ros2 launch robot_bring_up serial.launch.py
```


> **关键参数检查清单：**
> - `target_point.x/y/z`：目标点坐标，需在建图后根据场地标注
> - `global_frame`：应为 `"map"`
> - `robot_base_frame`：应为 `"base_link"`
> - `is_we_are_blue`：红蓝方选择（在 `sentry.yaml` 中）

## 建图流程

1. **采集点云**（搜索 `map1`，修改相关参数）：
   ```bash
   ros2 launch robot_bring_up sentry.launch.py
   # if_map 设为 True，仅运行 LiDAR + Point-LIO
   ```

2. **合并 PCD**（如保存了多个点云文件）：
   - 将所有 PCD 放入 `Point-LIO/PCD/merge` 目录
   - 编译后运行：
   ```bash
   ros2 launch merge_pcd merge_pcd.launch.py
   ```

3. **PCD 转栅格地图**（搜索 `map2`，修改路径和名称）：
   ```bash
   ros2 launch robot_bring_up pcd2pgm.launch.py
   ```

4. **使用地图**（搜索 `map3`，修改 `yaml_filename` 路径）：
   ```bash
   ros2 launch robot_bring_up sentry.launch.py
   ```

## 参数配置

所有参数集中在 `robot_bring_up/config/sentry.yaml` 中管理。关键配置项：

### 定位参数 (Point-LIO)
- `is_we_are_blue`：红蓝方选择
- `init_trans_red/blue`：初始位置偏移
- `pcd_save_en`：是否保存点云（建图时为 `true`）
- `use_pcd_map_`：是否加载先验地图（导航时为 `true`）

### 导航参数 (Nav2)
- `controller_frequency`：控制器频率
- `xy_goal_tolerance`：目标到达判定阈值
- TEB 局部规划器参数详见配置文件注释

### 障碍物分割参数
- `obstacle_x/y/z_min/max`：障碍物检测范围
- `cluster_tolerance`：聚类距离阈值

## 串口通信协议

| 协议 ID | 方向 | 用途 |
|---------|------|------|
| `0x0501` | 上位机→下位机 | 底盘速度指令 |
| `0x0606` | 上位机→下位机 | 距离信息（功能1） |
| `0x0607` | 下位机→上位机 | 裁判系统机器人位置 |
| 导航命令 | 下位机→上位机 | command=1: 标记锚点; command=2: 导航至锚点; command=3: 导航至目标点 |

## 注意事项

1. **目标点标注**：目标点（敌方基地装甲板）坐标需在完成建图后手动标注到 `sentry.yaml` 中的 `navigation_manager` 参数里，可能存在人工标注误差。
2. **建图模式**：`sentry.launch.py` 中 `if_map = True` 时为建图模式，会跳过障碍物分割和导航栈，仅运行 LiDAR + Point-LIO。
3. **红蓝方切换**：需修改 `pointlio.yaml` 中的 `is_we_are_blue` 以及对应的地图文件路径。
4. **导航精度**：导航至传送锚点功能精度为 10-20cm，尚未针对英雄/步兵的精度要求调校，如需使用可后续优化。

## 许可证

详见各子包 `package.xml` 中的声明。
