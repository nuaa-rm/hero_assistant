# Sentry Strategy Service

## 概述

`sentry_strategy_service` 是RoboMaster哨兵机器人的策略服务包，负责处理哨兵机器人攻击的决策逻辑和策略执行。该服务基于ROS2框架构建，提供哨兵机器人的智能决策功能。

## 功能特性

- 哨兵机器人策略决策
- 敌方目标识别和跟踪
- 战术策略执行
- 与rm_interfaces消息系统集成
- 实时策略调整

## 主要组件

### 1. SentryStrategyService
核心策略服务类，负责：
- 处理敌方策略请求
- 执行战术决策
- 管理哨兵机器人状态
- 与下位机通信

### 2. SentryStrategyServiceNode
ROS2节点封装，提供：
- 服务接口
- 话题发布/订阅
- 参数管理
- 生命周期管理

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select sentry_strategy_service
```

### 2. 启动服务

```bash
# 启动哨兵策略服务
ros2 run sentry_strategy_service sentry_strategy_service_node
```

### 3. 使用服务

通过ROS2服务调用接口：

```bash
# 调用敌方策略服务
ros2 service call /sentry_strategy_service/enemy_strategist rm_interfaces/srv/EnemyStrategist
```

## 服务接口

### EnemyStrategist 服务

**服务名称**: `/sentry_strategy_service/enemy_strategist`

**服务类型**: `rm_interfaces/srv/EnemyStrategist`

**功能**: 处理敌方策略决策请求

**请求参数**:
- `std_msgs/Header header`: 消息头
- `EnemyPositions all_enemy`: 所有敌方位置信息

**响应参数**:
- `int8 err_code`: 错误代码
  - 0: 一切正常，听自瞄的（不启用id）
  - 1: 强制追踪某个目标（启用id）
  - 2: 当前关闭自瞄
- `string id`: 目标ID
- `EnemyPosition position`: 敌方位置信息

## 配置参数

服务支持以下配置参数：

- `strategy_mode`: 策略模式设置
- `target_priority`: 目标优先级
- `tracking_threshold`: 跟踪阈值
- `decision_frequency`: 决策频率

## 依赖

- ROS2 (Humble)
- rclcpp
- rm_interfaces
- geometry_msgs
- std_msgs

## 文件结构

```
sentry_strategy_service/
├── CMakeLists.txt                    # 构建配置
├── package.xml                       # 包信息
├── README.md                        # 本文档
├── include/
│   └── sentry_strategy_service/     # 头文件目录
├── src/
│   ├── sentry_strategy_service.cpp  # 核心服务实现
│   └── sentry_strategy_service_node.cpp # 节点封装
└── launch/                          # 启动文件目录
```

## 策略逻辑

### 决策流程

1. **目标识别**: 接收敌方位置信息
2. **策略分析**: 根据当前状态分析最优策略
3. **决策执行**: 执行选定的策略
4. **反馈调整**: 根据执行结果调整策略

### 策略模式

- **自动模式**: 完全自主决策
- **手动模式**: 人工干预决策
- **混合模式**: 自动决策与人工干预结合

## 故障排除

### 常见问题

1. **服务启动失败**
   ```bash
   # 检查依赖
   ros2 pkg list | grep sentry_strategy_service
   
   # 检查服务状态
   ros2 service list | grep enemy_strategist
   ```

2. **策略执行异常**
   ```bash
   # 查看节点日志
   ros2 run sentry_strategy_service sentry_strategy_service_node
   
   # 检查话题
   ros2 topic list | grep sentry
   ```

3. **消息通信问题**
   ```bash
   # 检查消息类型
   ros2 interface list | grep rm_interfaces
   
   # 测试服务调用
   ros2 service call /sentry_strategy_service/enemy_strategist rm_interfaces/srv/EnemyStrategist
   ```

## 调试

### 启用调试模式

```bash
# 设置日志级别
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1

# 启动服务
ros2 run sentry_strategy_service sentry_strategy_service_node
```

### 监控服务状态

```bash
# 查看服务信息
ros2 service info /sentry_strategy_service/enemy_strategist

# 监听服务调用
ros2 service echo /sentry_strategy_service/enemy_strategist
```

## 扩展

### 添加新策略

1. 在 `sentry_strategy_service.cpp` 中添加新策略逻辑
2. 更新服务接口（如需要）
3. 添加相应的配置参数
4. 更新文档

### 集成新传感器

1. 添加新的消息订阅
2. 更新策略决策逻辑
3. 测试集成效果

## 维护者

- 维护者: elsa

## 许可证

TODO: License declaration 