# Point-LIO

## 概述

`Point-LIO` 是RoboMaster项目中的激光雷达惯性里程计包，基于Point-LIO算法实现高精度的激光雷达惯性里程计。该包集成了激光雷达和IMU数据，提供实时的位姿估计和点云处理功能。

## 功能特性

- 激光雷达惯性里程计
- 实时位姿估计
- 点云预处理和滤波
- 高精度定位
- 实时建图
- 多传感器融合

## 主要组件

### 1. 点云处理模块
负责处理激光雷达原始数据：
- **customPoint**: livox_driver2发送的初始点云
- **pl_surf**: 通过p_pre预处理后的点云
  - 将有效的、根据point_filter_num下采样的点加入pl_surf
  - 符合blind条件的点云
  - 体素滤波过的点云
  - 根据CustomPoint的offset_time，给出每个点的精确时间戳curvature

### 2. 数据缓冲模块
负责管理点云数据缓冲：
- **lidar_buffer**: 通过livox_pcl_cbk处理点云
  - 选项1：将一帧点云按时间分开
  - 选项2：将连续con_frame_num帧点云存入lidar_buffer
  - 选项3：将每帧点云存入lidar_buffer

### 3. 数据同步模块
负责多传感器数据同步：
- **Measures**: 通过sync_packages，将lidar_buffer中的最早一份点云传给measures

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select Point-LIO
```

### 2. 启动节点

```bash
# 启动Point-LIO节点
ros2 launch Point-LIO point_lio.launch.py
```

### 3. 配置参数

所有参数均在 `robot_bring_up/config/sentry.yaml` 中集中管理和修改，主要包括：

```yaml
# 传感器配置
lidar_topic: "/livox/lidar"
imu_topic: "/livox/imu"
odom_topic: "/Odometry"

# 点云处理参数
point_filter_num: 1
voxel_leaf_size: 0.1
blind: 0.5

# 建图参数
pcd_save_en: false
interval: 900
```

## 话题接口

### 订阅的话题

- `/livox/lidar`: 激光雷达点云数据
- `/imu/data`: IMU数据
- `/tf`: 坐标变换信息
- `/tf_static`: 静态坐标变换信息

### 发布的话题

- `/point_lio/cloud_registered`: 注册后的点云
- `/point_lio/odometry`: 里程计信息
- `/point_lio/path`: 轨迹路径
- `/point_lio/map`: 建图结果

## 配置参数

### 传感器参数

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

- `lidar_topic`: 激光雷达话题（默认: "/livox/lidar"）
- `imu_topic`: IMU话题（默认: "/livox/imu"）
- `odom_topic`: 里程计话题（默认: "/Odometry"）

### 处理参数

- `point_filter_num`: 点云滤波参数（默认: 1）
- `blind`: 盲区参数（默认: 0.5）
- `voxel_leaf_size`: 体素滤波大小（默认: 0.1）
- `scan_line`: 扫描线数（默认: 4）
- `scan_rate`: 扫描频率（默认: 10）

### 建图参数

- `pcd_save_en`: 是否保存点云地图（默认: false）
- `interval`: 保存间隔（默认: 900）
- `use_pcd_map_`: 是否使用先验地图（默认: true）
- `prior_PCD_map_path`: 先验地图路径

## 文件结构

```
Point-LIO/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包信息
├── README.md              # 本文档
├── LICENSE                # 许可证文件
├── include/
│   └── Point-LIO/        # 头文件目录
├── src/
│   └── (源文件)          # 实现文件
├── launch/
│   └── point_lio.launch.py # 启动文件
├── config/
│   └── point_lio.yaml    # 配置文件
├── rviz_cfg/
│   └── point_lio.rviz    # RViz配置文件
└── Log/                  # 日志目录
```

## 算法流程

### 1. 数据预处理
```
原始点云 → 下采样 → 滤波 → 时间戳处理
```

### 2. 数据同步
```
激光雷达数据 + IMU数据 → 时间同步 → 数据融合
```

### 3. 位姿估计
```
同步数据 → 特征提取 → 位姿估计 → 轨迹更新
```

### 4. 建图输出
```
位姿估计 → 点云注册 → 地图更新 → 结果输出
```

## 性能优化

### 1. 处理速度优化

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

```yaml
# 增加体素大小
voxel_leaf_size: 0.15

# 减少扫描线数
scan_line: 2
```

### 2. 内存优化

```yaml
# 减少点云滤波参数
point_filter_num: 2

# 关闭点云保存
pcd_save_en: false
```

### 3. 精度优化

```yaml
# 调整盲区参数
blind: 0.3

# 增加扫描线数
scan_line: 6
```

## 故障排除

### 常见问题

1. **传感器数据丢失**
   ```bash
   # 检查传感器话题
   ros2 topic list | grep livox
   ros2 topic list | grep imu
   
   # 检查数据频率
   ros2 topic hz /livox/lidar
   ros2 topic hz /imu/data
   ```

2. **位姿估计不准确**
   ```bash
   # 检查坐标系变换
   ros2 run tf2_tools view_frames
   ```
   
   请在 `robot_bring_up/config/sentry.yaml` 中调整参数：
   ```yaml
   point_filter_num: 2
   blind: 0.3
   ```

3. **处理延迟过高**
   
   请在 `robot_bring_up/config/sentry.yaml` 中调整参数：
   ```yaml
   voxel_leaf_size: 0.15
   scan_line: 2
   ```

### 调试命令

```bash
# 查看话题信息
ros2 topic list | grep point_lio
ros2 topic info /point_lio/odometry

# 监听里程计数据
ros2 topic echo /point_lio/odometry

# 可视化轨迹
rviz2
# 添加Path显示，话题选择/point_lio/path
```

## 集成使用

### 与Navigation2集成

1. 使用Point-LIO的里程计作为导航系统的定位输入
2. 配置相应的坐标系变换
3. 调整导航参数以适应Point-LIO的输出

### 与SLAM集成

1. 使用Point-LIO的位姿估计作为SLAM的初始值
2. 配置多传感器融合
3. 调整SLAM参数

## 扩展

### 添加新的传感器

1. 在配置文件中添加新传感器参数
2. 更新数据同步模块
3. 添加相应的坐标系变换
4. 测试新传感器的集成

### 支持新的算法

1. 在算法模块中实现新算法
2. 添加相应的参数配置
3. 更新launch文件选项
4. 测试新算法的效果

## 维护者

- 维护者: cmyhj
- 邮箱: autism2484684043@163.com

## 许可证

LICENSE