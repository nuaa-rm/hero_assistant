# Obstacle Segmentation TC

## 概述

`obstacle_segmentation_tc` 是RoboMaster项目的障碍物分割包，负责对原始点云数据进行处理，识别和分割障碍物，为导航系统提供准确的障碍物信息。

## 功能特性

- 点云下采样和滤波
- 基于法向量的障碍物识别
- 地面去除算法
- 障碍物分割和标记
- 实时点云处理
- 为costmap提供障碍物层数据

## 主要功能

### 1. 点云预处理
- **下采样**: 减少点云密度，提高处理效率
- **滤波**: 去除噪声和异常点
- **体素滤波**: 统一点云密度

### 2. 障碍物识别
- **法向量计算**: 计算点云法向量
- **法线夹角分析**: 通过法线夹角判断是否为障碍物
- **地面检测**: 识别并去除地面点云

### 3. 障碍物分割
- **聚类分割**: 将障碍物点云分组
- **边界提取**: 提取障碍物边界
- **标记分类**: 为不同障碍物类型标记

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select obstacle_segmentation_tc
```

### 2. 启动节点

```bash
# 启动障碍物分割节点
ros2 launch obstacle_segmentation_tc obstacle_segmentation.launch.py
```

### 3. 配置参数

所有参数均在 `robot_bring_up/config/sentry.yaml` 中集中管理和修改，主要包括：

```yaml
# 输入输出话题
input_cloud_topic: "/cloud_registered_body"
output_cloud_topic: "/cloud_obstacle"

# 处理参数
use_downsample: true
leaf_size: 0.1
point_num_for_normal: 10
angle_threshold: 0.785

# 障碍物范围参数
obstacle_x_min: -10.0
obstacle_x_max: 10.0
obstacle_y_min: -10.0
obstacle_y_max: 10.0
obstacle_z_min: -1.0
obstacle_z_max: 0.02
obstacle_range_min: 0.12
obstacle_range_max: 15.0

# 聚类参数
cluster_tolerance: 0.1
min_cluster_size: 100
max_cluster_size: 25000
```

## 话题接口

### 订阅的话题

- `/cloud_in`: 输入点云话题
- `/tf`: 坐标变换信息
- `/tf_static`: 静态坐标变换信息

### 发布的话题

- `/obstacle_cloud`: 障碍物点云
- `/ground_cloud`: 地面点云
- `/segmented_cloud`: 分割后的点云
- `/obstacle_markers`: 障碍物标记（可视化）

## 配置参数

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

### 输入输出参数

- `input_cloud_topic`: 输入点云话题（默认: "/cloud_registered_body"）
- `output_cloud_topic`: 输出障碍物点云话题（默认: "/cloud_obstacle"）

### 处理参数

- `use_downsample`: 是否使用下采样（默认: true）
- `leaf_size`: 体素滤波大小（默认: 0.1）
- `point_num_for_normal`: 法向量计算点数（默认: 10）
- `angle_threshold`: 法线夹角阈值（默认: 0.785）

### 障碍物范围参数

- `obstacle_x_min/max`: 障碍物X轴范围（默认: -10.0 到 10.0）
- `obstacle_y_min/max`: 障碍物Y轴范围（默认: -10.0 到 10.0）
- `obstacle_z_min/max`: 障碍物Z轴范围（默认: -1.0 到 0.02）
- `obstacle_range_min/max`: 障碍物距离范围（默认: 0.12 到 15.0）

### 聚类参数

- `cluster_tolerance`: 聚类距离阈值（默认: 0.1）
- `min_cluster_size`: 最小聚类大小（默认: 100）
- `max_cluster_size`: 最大聚类大小（默认: 25000）

## 依赖

- ROS2 (Humble)
- PCL (Point Cloud Library)
- sensor_msgs
- geometry_msgs
- visualization_msgs
- tf2
- tf2_ros

## 文件结构

```
obstacle_segmentation_tc/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包信息
├── README.md              # 本文档
├── include/
│   └── obstacle_segmentation_tc/ # 头文件目录
├── src/
│   └── (源文件)           # 实现文件
├── launch/
│   └── obstacle_segmentation.launch.py # 启动文件
└── config/
    └── (配置文件)         # 配置文件
```

## 算法流程

### 1. 数据预处理
```
原始点云 → 下采样 → 滤波 → 体素化
```

### 2. 法向量计算
```
滤波后点云 → 法向量计算 → 法线夹角分析
```

### 3. 障碍物分割
```
法向量分析 → 地面去除 → 障碍物聚类 → 边界提取
```

### 4. 输出处理
```
分割结果 → 标记生成 → 话题发布
```

## 性能优化

### 1. 参数调优

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

```yaml
# 调整体素大小以提高处理速度
leaf_size: 0.15

# 调整法向量计算点数
point_num_for_normal: 15
```

### 2. 内存优化

```yaml
# 减少聚类大小限制
max_cluster_size: 10000

# 调整障碍物范围
obstacle_range_max: 10.0
```

### 3. 精度优化

```yaml
# 调整聚类参数
min_cluster_size: 50
cluster_tolerance: 0.05

# 调整法线夹角阈值
angle_threshold: 0.6
```

## 故障排除

### 常见问题

1. **点云处理缓慢**
   
   请在 `robot_bring_up/config/sentry.yaml` 中调整参数：
   ```yaml
   # 增加体素大小
   leaf_size: 0.2
   
   # 减少法向量计算点数
   point_num_for_normal: 5
   ```

2. **障碍物识别不准确**
   
   请在 `robot_bring_up/config/sentry.yaml` 中调整参数：
   ```yaml
   # 调整法线夹角阈值
   angle_threshold: 0.6
   
   # 调整障碍物范围
   obstacle_z_max: 0.05
   ```

3. **内存使用过高**
   
   请在 `robot_bring_up/config/sentry.yaml` 中调整参数：
   ```yaml
   # 减少聚类大小限制
   max_cluster_size: 5000
   
   # 缩小障碍物范围
   obstacle_range_max: 8.0
   ```

### 调试命令

```bash
# 查看话题信息
ros2 topic list | grep cloud
ros2 topic info /obstacle_cloud

# 监听点云数据
ros2 topic echo /obstacle_cloud

# 可视化点云
rviz2
# 添加PointCloud2显示，话题选择/obstacle_cloud
```

## 集成使用

### 与Navigation2集成

1. 在costmap配置中添加障碍物层
2. 订阅障碍物点云话题
3. 配置相应的坐标系变换

### 与nav2_behaviors集成

为`nav2_behaviors/KeepAwayFromObstacles`插件提供障碍物信息：

```yaml
# 在behavior_tree中配置
- plugin: nav2_behaviors/KeepAwayFromObstacles
  inputs:
    obstacle_topic: /obstacle_cloud
```

## 扩展

### 添加新的分割算法

1. 在头文件中声明新的分割函数
2. 在源文件中实现算法
3. 更新参数配置
4. 测试算法效果

### 支持新的传感器

1. 添加新的点云输入话题
2. 实现传感器特定的预处理
3. 更新坐标系配置
4. 测试传感器兼容性

## 维护者

- 维护者: cmyhj
- 邮箱: autism2484684043@163.com

## 许可证

TODO: License declaration
