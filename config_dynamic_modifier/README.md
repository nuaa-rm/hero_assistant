# 配置动态修改器 (Config Dynamic Modifier)

这是一个ROS2节点，用于通过接收RViz2的Publish Point话题消息，动态修改配置文件中的参数。**现仅支持自动模式**，操作更简洁。

## 功能特性
- 接收RViz2的点击点消息
- 动态修改YAML配置文件中的参数
- 支持多个区域的配置修改
- 实时状态反馈
- 自动模式：根据预设点数自动完成多边形定义并自动切换区域

## 安装和编译

1. 将此包放在您的工作空间的src目录下
2. 编译工作空间：
```bash
colcon build --packages-select config_dynamic_modifier
source install/setup.bash
```

## 使用方法

### 1. 启动节点
```bash
ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py
# 或直接运行节点
ros2 run config_dynamic_modifier config_dynamic_modifier_node
```

### 2. 在RViz2中配置
1. 启动RViz2
2. 添加"Publish Point"工具
3. 确保点话题设置为 `/clicked_point`（默认）

### 3. 操作流程（自动模式）
1. 启动节点后，直接在RViz2中连续点击点
2. 系统会根据配置文件中`parameter_points_count`预设的点数自动完成多边形定义
3. 每个区域完成后自动切换到下一个区域，循环往复
4. 可通过监听`/config_modifier/status`话题获取状态反馈

### 4. 监听状态消息
```bash
ros2 topic echo /config_modifier/status
```

## 参数说明

| 参数名 | 说明 |
|--------|------|
| config_file_path | 配置文件路径 |
| point_topic | RViz2点话题 |
| status_topic | 状态话题 |
| parameter_points_count | 每个区域需要的点数（前4个为removed_area，后8个为tunnel） |

## 修改的参数

### Tunnel参数（后8个）
- `red_outpost_tunnel_x`, `red_outpost_tunnel_y`
- `blue_outpost_tunnel_x`, `blue_outpost_tunnel_y`
- `red_banana_tunnel_x`, `red_banana_tunnel_y`
- `blue_banana_tunnel_x`, `blue_banana_tunnel_y`

### Removed Area参数（前4个）
- `removed_area_1`
- `removed_area_2` 
- `removed_area_3`
- `removed_area_4`

这些参数定义了costmap中要移除的区域，格式为多边形坐标数组。

## 注意事项
1. 确保配置文件路径正确且有写入权限
2. 修改配置文件后，可能需要重新启动相关节点才能生效
3. 多边形坐标使用map坐标系
4. 建议在修改前备份原始配置文件
5. 自动模式下，系统会永久等待用户点击足够数量的点后再切换区域

## 故障排除

1. **无法打开配置文件**
   - 检查文件路径是否正确
   - 确保有读取和写入权限
2. **参数未找到**
   - 检查配置文件中是否存在对应的参数名
   - 确保参数格式正确
3. **RViz2点击无响应**
   - 检查点话题是否正确设置
   - 确保RViz2中的Publish Point工具已启用

---

如有问题请查阅`QUICK_START_GUIDE.md`或联系维护者。 