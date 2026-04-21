# Robot Serial

## 概述

`robot_serial` 是RoboMaster项目的串口通信包，负责机器人上位机与下位机之间的串口通信。该包基于ROS2框架构建，提供标准化的串口通信接口。

## 功能特性

- 串口通信管理
- 消息序列化和反序列化
- 与下位机实时通信
- 支持多种消息类型
- 自动重连机制
- 错误处理和恢复

## 主要组件

### 1. 串口通信模块
负责：
- 串口设备管理
- 数据读写
- 连接状态监控
- 错误处理

### 2. 消息处理模块
负责：
- 消息编码/解码
- 协议解析
- 数据验证
- 消息路由

### 3. ROS2接口模块
负责：
- 话题发布/订阅
- 服务调用
- 参数管理
- 节点生命周期

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select robot_serial
```

### 2. 启动节点

```bash
# 启动串口通信节点
ros2 run robot_serial robot_serial_node
```

### 3. 配置串口参数

所有参数均在 `robot_bring_up/config/sentry.yaml` 中集中管理和修改，主要包括：

```yaml
# 串口通信参数
dog_threshold: 100
frequency: 50.0
```

## 消息类型

### 发布的话题

- `/robot_status`: 机器人状态信息
- `/sensor_data`: 传感器数据
- `/motor_feedback`: 电机反馈信息

### 订阅的话题

- `/robot_command`: 机器人控制命令
- `/motor_command`: 电机控制命令
- `/gimbal_command`: 云台控制命令

### 服务

- `/serial_status`: 串口状态查询服务
- `/connection_test`: 连接测试服务

## 配置参数

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

### 串口通信参数

- `dog_threshold`: 看门狗阈值（默认: 100）
- `frequency`: 通信频率（默认: 50.0 Hz）

### 通信配置

- 串口设备路径、波特率等硬件参数需在代码中配置
- 通信协议和消息格式在代码中定义
- 错误处理和重连机制在代码中实现

## 依赖

- ROS2 (Humble)
- std_msgs
- sensor_msgs
- nav_msgs
- tf2
- geometry_msgs
- rm_interfaces

## 文件结构

```
robot_serial/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包信息
├── README.md              # 本文档
├── include/
│   └── robot_serial/      # 头文件目录
├── src/
│   └── (源文件)           # 实现文件
└── thirdparty/            # 第三方库
```

## 通信协议

### 消息格式

```
[Header][Length][Data][Checksum]
```

- **Header**: 消息头标识
- **Length**: 数据长度
- **Data**: 实际数据
- **Checksum**: 校验和

### 消息类型

1. **状态消息**: 机器人状态反馈
2. **控制消息**: 控制命令
3. **配置消息**: 参数配置
4. **心跳消息**: 连接保活


## 维护者

- 维护者: mijiao
- 邮箱: 1012866210@qq.com

## 许可证

TODO: License declaration
