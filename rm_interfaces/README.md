# RM Interfaces

## 概述

`rm_interfaces` 是RoboMaster项目的消息接口包，定义了项目中各个节点之间通信所需的自定义消息类型和服务。

## 功能特性

- 定义自定义消息类型（Message）
- 定义自定义服务类型（Service）
- 为RoboMaster哨兵机器人提供标准化的通信接口
- 支持ROS2消息系统

## 消息类型

### 服务 (Services)

#### EnemyStrategist.srv
用于哨兵机器人的敌方策略服务，包含：

**请求 (Request):**
- `std_msgs/Header header`: 消息头
- `EnemyPositions all_enemy`: 所有敌方位置信息

**响应 (Response):**
- `int8 err_code`: 错误代码
  - 0: 一切正常，听自瞄的（不启用id）
  - 1: 强制追踪某个目标（启用id）
  - 2: 当前关闭自瞄
- `string id`: 目标ID
- `EnemyPosition position`: 敌方位置信息

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select rm_interfaces
```

### 2. 设置环境

```bash
# 设置环境变量
source install/setup.bash
```

### 3. 在其他包中使用

在需要使用这些消息类型的包的 `package.xml` 中添加依赖：

```xml
<depend>rm_interfaces</depend>
```

在C++代码中包含头文件：

```cpp
#include "rm_interfaces/srv/enemy_strategist.hpp"
```

## 依赖

- ROS2 (Humble)
- std_msgs
- geometry_msgs
- rosidl_default_generators
- rosidl_default_runtime

## 文件结构

```
rm_interfaces/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包信息
├── README.md              # 本文档
├── msg/                   # 消息定义目录
│   └── (消息文件)
└── srv/                   # 服务定义目录
    └── EnemyStrategist.srv # 敌方策略服务
```

## 扩展

如需添加新的消息类型：

1. 在 `msg/` 目录下创建新的 `.msg` 文件
2. 在 `srv/` 目录下创建新的 `.srv` 文件
3. 更新 `CMakeLists.txt` 和 `package.xml`
4. 重新编译

## 维护者

- 维护者: elsa
- 邮箱: 2272268276@qq.com

## 许可证

TODO: License declaration 