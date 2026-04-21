# Auto Sentry 2025

## 项目概述

Auto Sentry 2025 是一个基于ROS2的RoboMaster哨兵机器人自主导航系统。该项目集成了先进的感知、定位、导航和决策技术，为RoboMaster比赛中的哨兵机器人提供完整的自主解决方案。

## 主要特性

- **智能导航**: 基于Navigation2的高精度导航系统
- **多传感器融合**: 激光雷达、IMU、相机等多传感器数据融合
- **动态配置**: 支持运行时动态修改导航参数
- **行为决策**: 基于行为树的智能决策系统
- **实时感知**: 高精度障碍物检测和分割
- **模块化设计**: 高度模块化的系统架构

## 系统架构

### 核心模块

1. **感知系统**
   - `obstacle_segmentation_tc`: 障碍物,地面分割
   - `lidar_merge`: 多激光雷达数据合并(后已合并到Point-LIO中，不再独立使用)
   - `Point-LIO`: 激光雷达惯性里程计定位

2. **导航系统**
   - `navigation2`: ROS2导航框架
   - `nav2_custom_costmap_plugin`: 自定义代价地图插件，将该地图部分代价强制清零，用于过狗洞
   - `config_dynamic_modifier`: 动态配置修改器，用于快速标记特殊区域（狗洞范围和狗洞需清零位置），并写入yaml，
                                便于在建完图后能快速标点并测试过洞

3. **决策系统**
   - `robot_behavior_tree`: 行为树决策系统，详细介绍见[README.md](robot_behavior_tree/README.md)
   - `sentry_strategy_service`: 哨兵击打策略服务，用于配合自瞄和全向感知做选车逻辑

4. **通信系统**
   - `robot_serial`: 串口通信
   - `rm_interfaces`: 消息接口定义，rm_interfaces包名和自瞄统一

5. **启动系统**
   - `robot_bring_up`: 系统启动配置

6. **建图系统**
   - `merge_pcd`: 将建图得到的所有.pcd文件合并
   - `pcd2pgm`: 从.pcd文件中生成栅格地图

## 快速开始

### 1. 环境要求

- Ubuntu 22.04
- ROS2 Humble
- C++14 或更高版本
- Python 3.8 或更高版本

### 2. 安装依赖

详细环境安装流程请见[《哨兵环境重装流程》](./哨兵环境重装流程/哨兵环境重装流程%2016c24a7052ae80909490d654ea911af2.md)

### 3. 启动系统

```bash
# 启动定位和导航相关模块(雷达驱动,点云融合,point-lio,navigation,点云分割,哨兵攻击决策等)
ros2 launch robot_bring_up sentry.launch.py

# 启动决策
ros2 launch robot_bt_decision_maker robot_bt_decision_maker.launch.py

# 启动串口
ros2 launch robot_bring_up serial.launch.py
```
建图流程请见[mapping.md](./mapping.md)
## 包说明

### 核心功能包

| 包名 | 功能 | 状态 |
|------|------|------|
| `config_dynamic_modifier` | 动态配置修改器 | ✅ 完成 |
| `rm_interfaces` | 消息接口定义 | ✅ 完成 |
| `sentry_strategy_service` | 哨兵策略服务 | ✅ 完成 |
| `robot_serial` | 串口通信 | ✅ 完成 |
| `obstacle_segmentation_tc` | 障碍物分割 | ✅ 完成 |
| `robot_bring_up` | 系统启动配置 | ✅ 完成 |
| `lidar_merge` | 激光雷达合并 | ✅ 完成 |
| `Point-LIO` | 激光雷达惯性里程计 | ✅ 完成 |
| `nav2_custom_costmap_plugin` | 自定义代价地图插件 | ✅ 完成 |
| `robot_behavior_tree` | 行为树决策系统 | ✅ 完成 |

### 第三方包

| 包名 | 功能 | 状态 |
|------|------|------|
| `navigation2` | ROS2导航框架 | ✅ 集成 |
| `teb_local_planner` | 局部路径规划器 | ✅ 集成 |

## 使用指南

### 1. 配置动态修改器

```bash
# 启动定位和导航相关模块(雷达驱动,点云融合,point-lio,navigation,点云分割等)
ros2 launch robot_bring_up sentry.launch.py

# 启动配置修改器
ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py
```

## 故障排除

### 常见问题

1. **编译错误**
   ```bash
   # 清理并重新编译
   colcon build --cmake-clean-cache
   colcon build
   ```

2. **启动失败**
   ```bash
   # 检查依赖
   ros2 pkg list
   
   # 检查话题
   ros2 topic list
   ```

### 调试命令

```bash
# 查看节点状态
ros2 node list
ros2 node info /node_name

# 查看话题
ros2 topic list
ros2 topic echo /topic_name

# 查看服务
ros2 service list
ros2 service call /service_name service_type
```

## 开发指南

### 1. 添加新功能

1. 在相应包中添加新功能
2. 更新launch文件
3. 添加配置文件
4. 更新文档

### 2. 修改配置

1. 编辑相应的yaml配置文件
2. 重启相关节点
3. 测试配置效果

### 3. 调试技巧

1. 使用RViz2进行可视化(包括查看点云输出和路径规划输出)
2. 启用详细日志输出
3. 使用ros2工具进行调试

### 4.赛季问题
1. 过狗洞目前基本依靠调整全局地图和中间空白代价区域的方式来进行微调，在赛场上给我们P图的机会并不多，
导致如果建图和适应性没有调好会十分影响后续正赛过洞情况
2. 过狗洞时为了保证不把上方横杆当成障碍物而采取代价清零的方式，这种方式的确极大的提高了狗洞的通过性，
在规划路径时也会受0代价影响而优先规划过狗洞的路径，但弊端是即使有车堵在狗洞也会默认那边无障碍物
3. Point-LIO定位有时候会有0.07s左右的延时
4. 本赛季使用的Point-LIO是回退了一个版本的Point-LIO，因此没有很好的运动去畸变，在高速旋转的情况下点云也会跟着剧烈晃动，目前该情况会影响建图，
如果建图时进行高速旋转会导致合并后的点云有偏转一个角度的重影
5. Point-LIO的先验地图启动会出现启动飘飞情况，该情况在家里走廊测试时不出现，但在常州场地时经常出现，目前解决办法是不使用合并后的总点云，
改用在两边半场的部分点云作为先验地图。暂时未排查出飘飞原因，怀疑过pcd文件中点云过多和feats_down_size < 4时定位不稳等情况，但是似乎都不是问题根源
6. 避障效果较差，导航目前速度参数都是顶电控上限的参数，速度很快，有时会出现直接不避障的情况，怀疑和定位延迟有关系(第3点)，也有可能是nuc运行程序卡顿，掉帧的问题(有不少时候nuc会出现整个程序延时达到几秒钟的情况，包括点云图、定位等)，目前不太清楚是不是硬件问题，后续需要进一步排查
7. teb目前有针对控头、控底盘和小陀螺前进三种情况下的参数，动态参数切换功能为service，有极小极小的概率切换失败（非必要优化项）
8. 行为树的问题在行为树的README.md中
9. 全向感知和选车逻辑

## 贡献指南

1. Fork项目
2. 创建功能分支
3. 提交更改
4. 创建Pull Request

## 维护者

- **主要维护者**: cmyhj elsa
- **邮箱**: autism2484684043@163.com 

## 许可证

本项目采用Apache License 2.0许可证。

## 更新日志

### v1.0.0 (2025-01-XX)
- 初始版本发布
- 完整的导航系统
- 动态配置功能
- 多传感器融合
- 行为树决策系统

## 相关链接
- [25赛季哨兵组研发文档](https://www.notion.so/25-1c724a7052ae80cab1cef2d2abbf24ba?source=copy_link)<-- 看这里，有很多经验！
- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [Navigation2文档](https://navigation.ros.org/)
- [RoboMaster官方](https://www.robomaster.com/) # hero_assistant
