# Robot Bring Up

## 概述

`robot_bring_up` 是RoboMaster项目的机器人启动包，负责配置和启动整个机器人系统。该包包含所有必要的launch文件、配置文件和参数设置，用于快速启动和配置RoboMaster哨兵机器人。

## 功能特性

- 系统级启动配置
- 参数管理和配置
- 多节点协调启动
- 环境配置和初始化
- 调试和测试支持

## 主要组件

### 1. Launch文件
- **系统启动**: 启动完整的机器人系统
- **模块启动**: 启动特定的功能模块
- **测试启动**: 用于调试和测试的启动配置

### 2. 配置文件
- **参数配置**: 系统参数设置
- **传感器配置**: 传感器参数配置
- **导航配置**: 导航系统配置
- **通信配置**: 通信参数设置

### 3. 环境配置
- **坐标系配置**: TF变换配置
- **地图配置**: 地图和定位配置
- **硬件配置**: 硬件设备配置

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select robot_bring_up
```

### 2. 启动系统

#### 完整系统启动
```bash
# 启动完整的机器人系统
ros2 launch robot_bring_up robot_complete.launch.py
```

#### 模块化启动
```bash
# 启动导航系统
ros2 launch robot_bring_up navigation.launch.py

# 启动感知系统
ros2 launch robot_bring_up perception.launch.py

# 启动控制系统
ros2 launch robot_bring_up control.launch.py
```

#### 调试模式启动
```bash
# 启动调试模式
ros2 launch robot_bring_up debug.launch.py
```

## 配置文件

### 1. 系统配置

#### robot_complete.yaml
完整的系统参数配置：
```yaml
# 机器人基础配置
robot:
  name: "sentry_robot"
  type: "sentry"
  
# 传感器配置
sensors:
  lidar: true
  camera: true
  imu: true
  
# 导航配置
navigation:
  planner: "nav2_navfn_planner/NavfnPlanner"
  controller: "nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController"
  behavior_server: "nav2_behavior_server/BehaviorServer"
```

#### sentry.yaml
哨兵机器人特定配置：
```yaml
# 哨兵机器人配置
sentry:
  # 隧道参数
  red_outpost_tunnel_x: [1.0, 2.0, 3.0, 4.0]
  red_outpost_tunnel_y: [1.0, 2.0, 3.0, 4.0]
  blue_outpost_tunnel_x: [1.0, 2.0, 3.0, 4.0]
  blue_outpost_tunnel_y: [1.0, 2.0, 3.0, 4.0]
  red_banana_tunnel_x: [1.0, 2.0, 3.0, 4.0]
  red_banana_tunnel_y: [1.0, 2.0, 3.0, 4.0]
  blue_banana_tunnel_x: [1.0, 2.0, 3.0, 4.0]
  blue_banana_tunnel_y: [1.0, 2.0, 3.0, 4.0]
  
  # 移除区域参数
  removed_area_1: [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]
  removed_area_2: []
  removed_area_3: []
  removed_area_4: []
```

### 2. 导航配置

#### nav2_params.yaml
Navigation2参数配置：
```yaml
# 全局代价地图配置
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05

# 局部代价地图配置
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
```

## 许可证

TODO: License declaration 