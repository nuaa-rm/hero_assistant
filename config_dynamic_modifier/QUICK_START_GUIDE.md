# 配置动态修改器 - 快速开始指南

## 🚀 最简单的使用方法（自动模式）

### 步骤1：启动节点
```bash
ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py
```

### 步骤2：在RViz2中操作
- 打开RViz2，加载地图
- 添加"Publish Point"工具
- 直接点击地图上的点即可！

### 自动模式工作流程
- 按照配置文件`parameter_points_count`设定的点数，依次点击地图
- 每个区域点数达到后自动切换到下一个区域，循环往复
- 可通过监听`/config_modifier/status`话题获取状态反馈

## 📝 常用参数说明

| 参数名 | 说明 |
|--------|------|
| config_file_path | 配置文件路径 |
| point_topic | RViz2点话题 |
| status_topic | 状态话题 |
| parameter_points_count | 每个区域需要的点数（前4个为removed_area，后8个为tunnel） |

参数配置示例见 `config/config_dynamic_modifier.yaml`。

## 🛠 故障排除

1. **配置文件无法修改**
   - 检查文件权限
   - 检查文件路径
2. **点击无响应**
   - 检查点话题连接
   - 检查RViz2中"Publish Point"工具是否启用
3. **自动切换区域不工作**
   - 检查`parameter_points_count`参数设置

---

**总结**：现在只需启动节点，然后直接点击地图即可！自动模式让操作变得非常简单高效。 