#!/bin/bash

# 配置动态修改器 - 一键启动脚本
# 使用方法: ./quick_start.sh [选项]

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
AUTO_MODE=true
GUI_MODE=false
VOICE_MODE=false
GESTURE_MODE=false
DEBUG_MODE=false
CONFIG_FILE=""

# 显示帮助信息
show_help() {
    echo -e "${BLUE}配置动态修改器 - 一键启动脚本${NC}"
    echo ""
    echo "使用方法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -a, --auto-mode         启用自动模式 (默认)"
    echo "  -m, --manual-mode       启用手动模式"
    echo "  -g, --gui               启用GUI界面"
    echo "  -v, --voice             启用语音控制"
    echo "  -s, --gesture           启用手势控制"
    echo "  -d, --debug             启用调试模式"
    echo "  -c, --config FILE       指定配置文件路径"
    echo "  -r, --reset             重置所有区域"
    echo "  -l, --list-areas        列出当前区域状态"
    echo ""
    echo "示例:"
    echo "  $0                      # 启动自动模式"
    echo "  $0 -g                   # 启动GUI模式"
    echo "  $0 -m -d                # 启动手动调试模式"
    echo "  $0 -r                   # 重置所有区域"
}

# 检查ROS2环境
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未找到ROS2命令${NC}"
        echo "请确保已安装ROS2并已source环境"
        exit 1
    fi
}

# 检查工作空间
check_workspace() {
    if [ -z "$COLCON_PREFIX_PATH" ]; then
        echo -e "${YELLOW}警告: 未检测到colcon工作空间${NC}"
        echo "请运行: source install/setup.bash"
    fi
}

# 启动节点
start_node() {
    echo -e "${GREEN}启动配置动态修改器节点...${NC}"
    
    # 构建启动命令
    LAUNCH_CMD="ros2 launch config_dynamic_modifier config_dynamic_modifier.launch.py"
    
    # 添加参数
    if [ "$AUTO_MODE" = true ]; then
        LAUNCH_CMD="$LAUNCH_CMD auto_mode:=true"
    else
        LAUNCH_CMD="$LAUNCH_CMD auto_mode:=false"
    fi
    
    if [ "$GUI_MODE" = true ]; then
        LAUNCH_CMD="$LAUNCH_CMD gui_enabled:=true"
    fi
    
    if [ "$VOICE_MODE" = true ]; then
        LAUNCH_CMD="$LAUNCH_CMD voice_enabled:=true"
    fi
    
    if [ "$GESTURE_MODE" = true ]; then
        LAUNCH_CMD="$LAUNCH_CMD gesture_enabled:=true"
    fi
    
    if [ "$DEBUG_MODE" = true ]; then
        LAUNCH_CMD="$LAUNCH_CMD debug_mode:=true"
    fi
    
    if [ -n "$CONFIG_FILE" ]; then
        LAUNCH_CMD="$LAUNCH_CMD config_file_path:=$CONFIG_FILE"
    fi
    
    echo "执行命令: $LAUNCH_CMD"
    eval $LAUNCH_CMD
}

# 重置所有区域
reset_areas() {
    echo -e "${YELLOW}重置所有区域...${NC}"
    ros2 topic pub /config_modifier/command std_msgs/msg/String "data: 'reset'" --once
    echo -e "${GREEN}重置完成${NC}"
}

# 列出区域状态
list_areas() {
    echo -e "${BLUE}当前区域状态:${NC}"
    echo "区域0: removed_area_1"
    echo "区域1: removed_area_2"
    echo "区域2: removed_area_3"
    echo "区域3: removed_area_4"
    echo ""
    echo "使用以下命令查看状态:"
    echo "ros2 topic echo /config_modifier/status"
}

# 显示启动信息
show_startup_info() {
    echo -e "${GREEN}配置动态修改器已启动！${NC}"
    echo ""
    echo -e "${BLUE}使用方法:${NC}"
    if [ "$AUTO_MODE" = true ]; then
        echo "1. 在RViz2中添加'Publish Point'工具"
        echo "2. 直接点击地图上的点"
        echo "3. 连续点击4个点自动完成多边形"
    else
        echo "1. 发送命令选择区域: ros2 topic pub /config_modifier/command std_msgs/msg/String 'data: \"area 0\"'"
        echo "2. 在RViz2中点击地图上的点"
        echo "3. 发送命令完成: ros2 topic pub /config_modifier/command std_msgs/msg/String 'data: \"finish_polygon\"'"
    fi
    echo ""
    echo -e "${BLUE}监控状态:${NC}"
    echo "ros2 topic echo /config_modifier/status"
    echo ""
    echo -e "${BLUE}常用命令:${NC}"
    echo "重置所有区域: ros2 topic pub /config_modifier/command std_msgs/msg/String 'data: \"reset\"'"
    echo "重新加载costmap: ros2 topic pub /config_modifier/command std_msgs/msg/String 'data: \"reload\"'"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -a|--auto-mode)
            AUTO_MODE=true
            shift
            ;;
        -m|--manual-mode)
            AUTO_MODE=false
            shift
            ;;
        -g|--gui)
            GUI_MODE=true
            shift
            ;;
        -v|--voice)
            VOICE_MODE=true
            shift
            ;;
        -s|--gesture)
            GESTURE_MODE=true
            shift
            ;;
        -d|--debug)
            DEBUG_MODE=true
            shift
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        -r|--reset)
            check_ros2
            reset_areas
            exit 0
            ;;
        -l|--list-areas)
            list_areas
            exit 0
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 主程序
main() {
    echo -e "${BLUE}=== 配置动态修改器 - 一键启动脚本 ===${NC}"
    echo ""
    
    # 检查环境
    check_ros2
    check_workspace
    
    # 显示配置信息
    echo -e "${YELLOW}配置信息:${NC}"
    echo "自动模式: $AUTO_MODE"
    echo "GUI模式: $GUI_MODE"
    echo "语音控制: $VOICE_MODE"
    echo "手势控制: $GESTURE_MODE"
    echo "调试模式: $DEBUG_MODE"
    if [ -n "$CONFIG_FILE" ]; then
        echo "配置文件: $CONFIG_FILE"
    fi
    echo ""
    
    # 启动节点
    start_node &
    NODE_PID=$!
    
    # 等待节点启动
    sleep 2
    
    # 显示启动信息
    show_startup_info
    
    # 等待用户中断
    echo -e "${YELLOW}按 Ctrl+C 停止节点${NC}"
    wait $NODE_PID
}

# 运行主程序
main 