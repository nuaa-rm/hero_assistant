# 多边形模式使用指南

## 概述

配置动态修改器现在支持多种模式：
1. **自动模式**（推荐）：根据预设点数自动完成多边形定义
2. **手动模式**：手动控制多边形收集过程
3. **单点模式**：点击单个点生成矩形区域

## 自动模式使用方法（推荐）

### 1. 启动节点

```bash
# 启动节点（默认启用自动模式）
ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py
```

### 2. 在RViz2中操作

1. 启动RViz2
2. 添加"Publish Point"工具
3. 确保点话题设置为 `/clicked_point`
4. 直接在地图上连续点击点即可！

### 3. 自动模式工作流程

- **第1次点击**：开始收集多边形点
- **第2-3次点击**：继续添加点
- **第4次点击**：自动完成多边形并应用到当前区域
- **等待10秒**：自动切换到下一个区域
- **重复过程**：继续为下一个区域收集点

### 4. 区域切换

系统会自动在以下区域间切换：
- **区域0-7**：tunnel参数（red_outpost, blue_outpost, red_banana, blue_banana）
- **区域8-11**：removed_area参数（removed_area_1到removed_area_4）

## 手动模式使用方法

### 1. 启动节点

```bash
# 启动节点
ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py
```

### 2. 禁用自动模式

```bash
# 禁用自动模式
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'auto_mode off'"
```

### 3. 选择要修改的区域

```bash
# 选择区域 0-11
# 0-7为tunnel参数，8-11为removed_area参数
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'area 0'"
```

### 4. 开始多边形收集模式

```bash
# 开始收集多边形点
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'start_polygon'"
```

### 5. 在RViz2中点击点

1. 启动RViz2
2. 添加"Publish Point"工具
3. 确保点话题设置为 `/clicked_point`
4. 在地图上依次点击多个点来定义多边形
   - 至少需要3个点才能形成有效多边形
   - 建议按顺时针或逆时针顺序点击
   - 点击的点会按顺序连接形成多边形

### 6. 完成多边形定义

```bash
# 完成多边形并应用到配置文件
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'finish_polygon'"
```

### 7. 重新加载costmap（可选）

```bash
# 手动重新加载costmap
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'reload'"
```

## 完整示例

### 自动模式示例

```bash
# 1. 启动节点
ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py

# 2. 在RViz2中连续点击点
# 点击点1: (2.0, 3.0)
# 点击点2: (3.0, 3.0)
# 点击点3: (3.0, 2.0)
# 点击点4: (2.0, 2.0) - 自动完成并切换到下一个区域

# 3. 继续为下一个区域点击点...
```

### 手动模式示例

```bash
# 1. 选择区域0
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'area 0'"

# 2. 开始收集多边形点
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'start_polygon'"

# 3. 在RViz2中点击4个点形成矩形
# 点击点1: (2.0, 3.0)
# 点击点2: (3.0, 3.0)
# 点击点3: (3.0, 2.0)
# 点击点4: (2.0, 2.0)

# 4. 完成多边形定义
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'finish_polygon'"

# 5. 重新加载costmap
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'reload'"
```

## 其他有用命令

### 清除当前多边形点

```bash
# 清除当前收集的多边形点
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'clear_polygon'"
```

### 重置所有区域

```bash
# 重置所有区域为默认值
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'reset'"
```

### 启用/禁用自动模式

```bash
# 启用自动模式
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'auto_mode on'"

# 禁用自动模式
ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'auto_mode off'"
```

## 状态监控

### 监听状态消息

```bash
# 监听状态消息
ros2 topic echo /config_modifier/status
```

### 查看节点日志

```bash
# 查看节点日志
ros2 run config_dynamic_modifier config_dynamic_modifier_node
```

## 注意事项

### 1. 多边形点数要求
- 至少需要3个点才能形成有效多边形
- 可以通过参数 `min_polygon_points` 调整最小点数要求
- 每个区域需要的点数在 `parameter_points_count` 中配置

### 2. 点的顺序
- 点击的点的顺序决定了多边形的形状
- 建议按顺时针或逆时针顺序点击
- 最后一个点会自动与第一个点连接形成闭合多边形

### 3. 坐标系
- 所有坐标都使用map坐标系
- 确保RViz2中的坐标系设置正确

### 4. 错误处理
- 如果点数不足，会显示警告消息
- 如果配置文件路径错误，会显示错误消息
- 所有操作都有状态反馈

### 5. 自动模式延迟
- 自动模式下有10秒延迟，避免误操作
- 可以通过 `auto_switch_delay` 参数调整

## 高级功能

### 1. 多参数支持
- **Tunnel参数**（区域0-7）：
  - red_outpost_tunnel_x/y
  - blue_outpost_tunnel_x/y
  - red_banana_tunnel_x/y
  - blue_banana_tunnel_x/y
- **Removed Area参数**（区域8-11）：
  - removed_area_1到removed_area_4

### 2. 智能区域切换
- 自动模式下自动切换到下一个区域
- 支持12个不同的区域（0-11）
- 每个区域可以定义不同的多边形

### 3. 实时状态反馈
- 所有操作都有详细的状态反馈
- 显示当前收集的点数
- 显示操作结果和错误信息
- 显示当前区域和模式

### 4. GUI界面支持
- 提供RViz2面板插件
- 支持图形化操作
- 实时状态显示

## 故障排除

### 1. 点数不足错误
```
多边形点数不足，至少需要 3 个点，当前有 2 个点
```
**解决方案**: 继续点击更多点，或使用 `clear_polygon` 重新开始

### 2. 配置文件路径错误
```
无法打开配置文件: /path/to/config.yaml
```
**解决方案**: 检查配置文件路径是否正确，确保有读写权限

### 3. 区域索引错误
```
无效的区域索引，请输入 0-11
```
**解决方案**: 使用正确的区域索引（0-11）

### 4. RViz2点击无响应
**解决方案**: 
1. 检查点话题是否正确设置为 `/clicked_point`
2. 确保RViz2中的Publish Point工具已启用
3. 检查坐标系设置

### 5. 自动模式不工作
**解决方案**:
1. 检查自动模式是否启用
2. 确认配置文件中设置了正确的点数
3. 检查延迟设置

## 最佳实践

### 1. 多边形设计
- 使用尽可能少的点来定义简单多边形
- 避免过于复杂的多边形形状
- 确保多边形不会重叠或产生冲突

### 2. 工作流程
1. 先规划好要定义的多边形形状
2. 按顺序点击关键点
3. 完成定义后立即测试效果
4. 根据需要调整和重新定义

### 3. 测试和验证
- 每次修改后都要测试导航效果
- 使用RViz2可视化costmap变化
- 记录有效的多边形配置

### 4. 自动模式使用
- 推荐使用自动模式简化操作
- 提前规划好所有区域的多边形
- 利用延迟时间检查当前操作

## 配置参数

### 主要参数
```yaml
auto_mode: true                    # 启用自动模式
auto_switch_delay: 10.0            # 自动模式切换延迟（秒）
polygon_collection_mode: true      # 多边形收集模式
min_polygon_points: 3              # 最小多边形点数
parameter_points_count: [4, 4, 4, 4, 6, 6, 6, 6, 4, 6, 4, 6]  # 每个区域需要的点数
```

## 总结

多边形模式提供了更灵活的区域定义方式，允许用户通过点击多个点来精确控制要移除的区域形状。自动模式进一步简化了操作流程，特别适合快速配置多个区域。这种方法比固定大小的矩形更加灵活和精确，特别适用于复杂环境中的导航优化和RoboMaster比赛中的场地配置调整。 