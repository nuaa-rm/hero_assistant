# nav2_custom_costmap_plugin

## 简介

`nav2_custom_costmap_plugin` 是一个基于 ROS2 Navigation2 的自定义代价地图层插件，可在运行时将指定多边形区域设置为 FREE_SPACE，实现动态移除障碍物区域。

## 主要功能
- 通过参数指定多边形区域，在 costmap 中动态移除（置为 FREE_SPACE）。
- 支持最多 4 个移除区域。
- 插件可动态加载，兼容 nav2_costmap_2d 框架。

## 依赖
- ROS2 (建议 Foxy 及以上)
- nav2_costmap_2d
- pluginlib
- geometry_msgs
- rclcpp

## 编译方法
```bash
colcon build --packages-select nav2_custom_costmap_plugin
source install/setup.bash
```

## 插件使用方法
1. 在 `costmap` 的参数文件中添加插件：
```yaml
plugins:
  - {name: custom_layer, type: "nav2_custom_costmap_plugin::CustomLayer"}
```
2. 可选参数（可在 costmap yaml 文件中配置）：
```yaml
custom_layer:
  enabled: true
  removed_area_1: "[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]"
  removed_area_2: "[]"
  removed_area_3: "[]"
  removed_area_4: "[]"
```
- `removed_area_1~4`：字符串格式的多边形点集，格式同 footprint。
- `enabled`：是否启用该层。

## 参数说明
- `enabled` (bool)：是否启用该层，默认 true。
- `removed_area_1~4` (string)：多边形区域，格式为 `[[x1, y1], [x2, y2], ...]`，最多支持 4 个区域。

## 示例
```yaml
custom_layer:
  enabled: true
  removed_area_1: "[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]"
  removed_area_2: "[]"
  removed_area_3: "[]"
  removed_area_4: "[]"
```

## 插件注册
插件描述文件：`custom_layer.xml`
```xml
<library path="nav2_custom_costmap_plugin_core">
  <class name="nav2_custom_costmap_plugin::CustomLayer" type="nav2_custom_costmap_plugin::CustomLayer" base_class_type="nav2_costmap_2d::Layer">
    <description>This is an plugin which make the target areas free</description>
  </class>
</library>
```

## 维护者
- Alexey Merzlyakov <alexey.merzlyakov@samsung.com>
- 二次开发/维护：请补充 