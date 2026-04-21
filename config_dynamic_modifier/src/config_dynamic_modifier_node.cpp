#include "config_dynamic_modifier/config_dynamic_modifier.hpp"
#include <iostream>
#include <filesystem>

/**
 * @file config_dynamic_modifier_node.cpp
 * @brief 配置动态修改器节点实现文件，实现自动模式下的多边形区域收集与配置文件动态写入。
 */

namespace config_dynamic_modifier
{

/**
 * @class ConfigDynamicModifier
 * @brief ROS2节点，实现自动收集多边形区域并动态修改配置文件。
 */

ConfigDynamicModifier::ConfigDynamicModifier()
: Node("config_dynamic_modifier_node"),
  current_area_index_(0),
  click_count_(0)
{
  // 声明参数
  this->declare_parameter("config_file_path", "/home/tc/Desktop/auto_sentry2025/src/auto_sentry2025/robot_bring_up/config/sentry.yaml");
  this->declare_parameter("point_topic", "/clicked_point");
  this->declare_parameter("status_topic", "/config_modifier/status");
  this->declare_parameter<std::vector<int>>("parameter_points_count", {2, 3, 2, 3, 4, 4, 4, 4, 6, 6, 6, 6});

  // 获取参数
  this->get_parameter("config_file_path", config_file_path_);
  this->get_parameter("point_topic", point_topic_);
  this->get_parameter("status_topic", status_topic_);
  this->get_parameter<std::vector<long int>>("parameter_points_count", parameter_points_count_);
  if (parameter_points_count_.size() != parameter_names_.size()) {
    parameter_points_count_ = {2, 3, 2, 3, 4, 4, 4, 4, 6, 6, 6, 6};
  }

  // 初始化参数名称映射
  parameter_names_ = {
    // removed_area参数（前4个）
    "removed_area_1", "removed_area_2", "removed_area_3", "removed_area_4",
    // tunnel参数（后8个）
    "red_outpost_tunnel_x", "red_outpost_tunnel_y",
    "blue_outpost_tunnel_x", "blue_outpost_tunnel_y",
    "red_banana_tunnel_x", "red_banana_tunnel_y",
    "blue_banana_tunnel_x", "blue_banana_tunnel_y"
  };
  
  // 初始化当前多边形数据
  current_polygons_.resize(4);
  for (int i = 0; i < 4; ++i) {
    current_polygons_[i] = {{1.0, 1.0}, {0.5, 1.0}, {0.5, 0.5}, {1.25, 0.5}};
  }

  // 创建订阅者
  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    point_topic_, 10, 
    std::bind(&ConfigDynamicModifier::pointCallback, this, std::placeholders::_1));

  // 创建发布者
  status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "配置动态修改器节点已启动");
  RCLCPP_INFO(this->get_logger(), "配置文件路径: %s", config_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "点话题: %s", point_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "当前选中区域: %d", current_area_index_);
  RCLCPP_INFO(this->get_logger(), "仅自动模式可用");
  RCLCPP_INFO(this->get_logger(), "使用方法:");
  RCLCPP_INFO(this->get_logger(), "1. 在RViz2中publish点话题");
  RCLCPP_INFO(this->get_logger(), "2. 连续点击指定数量的点自动形成多边形");
}

/**
 * @brief 点话题回调函数，处理自动模式下的点收集。
 * @param msg 点消息
 */
void ConfigDynamicModifier::pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "收到点消息: x=%.2f, y=%.2f, z=%.2f", 
               msg->point.x, msg->point.y, msg->point.z);
  // 仅自动模式
  handleAutoMode(msg->point);
  // 发布状态消息
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "自动模式 - 已添加点到多边形，当前点数: " + std::to_string(current_polygon_points_.size()) + 
                   " (位置: " + std::to_string(msg->point.x) + ", " + std::to_string(msg->point.y) + ")";
  status_pub_->publish(status_msg);
}

/**
 * @brief 自动模式下处理点的逻辑，收集多边形并自动切换区域。
 * @param point 新增的点
 */
void ConfigDynamicModifier::handleAutoMode(const geometry_msgs::msg::Point& point)
{
  click_count_++;
  addPointToPolygon(point);
  int required_points = 4;
  if (current_area_index_ >= 0 && current_area_index_ < (int)parameter_points_count_.size()) {
    required_points = parameter_points_count_[current_area_index_];
  }
  RCLCPP_INFO(get_logger(), "required_point:%d", required_points);
  if ((int)current_polygon_points_.size() >= required_points) {
    finishPolygon();
    click_count_ = 0;
    // 自动切换到下一个区域
    current_area_index_ = (current_area_index_ + 1) % parameter_names_.size();
    RCLCPP_INFO(this->get_logger(), "自动切换到下一个区域: %d", current_area_index_);
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "已自动切换到下一个区域: " + std::to_string(current_area_index_);
    status_pub_->publish(status_msg);
  }
}

/**
 * @brief 向当前多边形添加一个点。
 * @param point 新增的点
 */
void ConfigDynamicModifier::addPointToPolygon(const geometry_msgs::msg::Point& point)
{
  current_polygon_points_.push_back(point);
  RCLCPP_INFO(this->get_logger(), "已添加点到多边形，当前点数: %lu", current_polygon_points_.size());
}

/**
 * @brief 清空当前多边形点。
 */
void ConfigDynamicModifier::clearCurrentPolygon()
{
  current_polygon_points_.clear();
  click_count_ = 0;
  RCLCPP_INFO(this->get_logger(), "已清除多边形点");
}

/**
 * @brief 完成当前多边形的收集，写入配置并切换区域。
 */
void ConfigDynamicModifier::finishPolygon()
{
  int required_points = 4;
  if (current_area_index_ >= 0 && current_area_index_ < (int)parameter_points_count_.size()) {
    required_points = parameter_points_count_[current_area_index_];
  }
  if ((int)current_polygon_points_.size() < required_points) {
    RCLCPP_WARN(this->get_logger(), "点数不足，至少需要 %d 个点，当前有 %lu 个点", 
                 required_points, current_polygon_points_.size());
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "点数不足，至少需要 " + std::to_string(required_points) + 
                     " 个点，当前有 " + std::to_string(current_polygon_points_.size()) + " 个点";
    status_pub_->publish(status_msg);
    return;
  }
  updateCostmapParameters(current_polygon_points_, current_area_index_);
  current_polygon_points_.clear();
  click_count_ = 0;
  RCLCPP_INFO(this->get_logger(), "已完成区域 %d 的点收集", current_area_index_);
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "已完成区域 " + std::to_string(current_area_index_) + " 的点收集";
  status_pub_->publish(status_msg);
}

/**
 * @brief 根据点生成多边形字符串（用于yaml写入）。
 * @param points 点列表
 * @return 多边形字符串
 */
std::string ConfigDynamicModifier::generatePolygonStringFromPoints(const std::vector<geometry_msgs::msg::Point>& points)
{
  // 将points视为中心线，生成多边形顶点
  std::vector<std::pair<double, double>> polygon_points;
  double offset = 0.01;
  // 先正向遍历，x+0.01
  for (const auto& pt : points) {
    polygon_points.emplace_back(pt.x + offset, pt.y);
  }
  // 再反向遍历，x-0.01
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    polygon_points.emplace_back(it->x - offset, it->y);
  }
  // 生成字符串
  std::ostringstream oss;
  oss << "[ ";
  for (size_t i = 0; i < polygon_points.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << "[ " << polygon_points[i].first << ", " << polygon_points[i].second << " ]";
  }
  oss << " ]";
  return oss.str();
}

/**
 * @brief 修改yaml配置文件中指定参数的值。
 * @param config_file_path 配置文件路径
 * @param parameter_name 参数名
 * @param new_value 新值
 * @param area_index 区域索引
 * @return 是否修改成功
 */
bool ConfigDynamicModifier::modifyConfigFile(const std::string& config_file_path, 
                                           const std::string& parameter_name, 
                                           const std::string& new_value,
                                           const int area_index)
{
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开配置文件: %s", config_file_path.c_str());
    return false;
  }

  std::string content;
  std::string line;
  
  // 读取文件内容
  while (std::getline(file, line)) {
    content += line + "\n";
  }
  file.close();

  // 构建正则表达式模式
  std::string pattern = "(\\s*" + parameter_name + "\\s*:\\s*)\"[^\"]*\"";
  std::string replacement = "$1\"" + new_value + "\"";
  if (area_index>3)
  {      
    pattern = "(\\s*" + parameter_name + "\\s*:\\s*)\\[[^\\]]*\\]";
    replacement = "$1" + new_value;
  }

  std::regex regex_pattern(pattern);
  
  // 替换参数值
  std::string new_content = std::regex_replace(content, regex_pattern, replacement);
  
  // 检查是否找到了参数
  if (new_content == content) {
    RCLCPP_WARN(this->get_logger(), "未找到参数: %s", parameter_name.c_str());
    return false;
  }

  // 写回文件
  std::ofstream out_file(config_file_path);
  if (!out_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法写入配置文件: %s", config_file_path.c_str());
    return false;
  }
  
  out_file << new_content;
  out_file.close();
  
  RCLCPP_INFO(this->get_logger(), "已更新参数 %s = %s", parameter_name.c_str(), new_value.c_str());
  return true;
}

/**
 * @brief 解析多边形字符串为点对vector。
 * @param polygon_str 多边形字符串
 * @return 点对vector
 */
std::vector<std::pair<double, double>> ConfigDynamicModifier::parsePolygonString(const std::string& polygon_str)
{
  std::vector<std::pair<double, double>> polygon;
  
  // 简单的解析，假设格式为 "[ [x1, y1], [x2, y2], ... ]"
  std::regex point_regex("\\[\\s*([+-]?\\d*\\.?\\d+)\\s*,\\s*([+-]?\\d*\\.?\\d+)\\s*\\]");
  std::sregex_iterator iter(polygon_str.begin(), polygon_str.end(), point_regex);
  std::sregex_iterator end;
  
  for (; iter != end; ++iter) {
    double x = std::stod((*iter)[1]);
    double y = std::stod((*iter)[2]);
    polygon.emplace_back(x, y);
  }
  
  return polygon;
}

/**
 * @brief 将点对vector转为字符串。
 * @param polygon 点对vector
 * @return 字符串
 */
std::string ConfigDynamicModifier::polygonToString(const std::vector<std::pair<double, double>>& polygon)
{
  std::ostringstream oss;
  oss << "[ ";
  for (size_t i = 0; i < polygon.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << "[ " << polygon[i].first << ", " << polygon[i].second << " ]";
  }
  oss << " ]";
  return oss.str();
}

/**
 * @brief 更新costmap相关参数并写入配置文件。
 * @param points 点列表
 * @param area_index 区域索引
 */
void ConfigDynamicModifier::updateCostmapParameters(const std::vector<geometry_msgs::msg::Point>& points, int area_index)
{
  if (area_index < 0 || area_index >= (int)parameter_names_.size()) {
    RCLCPP_ERROR(this->get_logger(), "无效的区域索引: %d", area_index);
    return;
  }
  if (points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "点列表为空");
    return;
  }
  // 前4个为removed_area参数，保持原有逻辑
  if (area_index < 4) {
    std::string new_polygon = generatePolygonStringFromPoints(points);
    current_polygons_[area_index] = parsePolygonString(new_polygon);
    std::string parameter_name = parameter_names_[area_index];
    if (modifyConfigFile(config_file_path_, parameter_name, new_polygon, area_index)) {
      RCLCPP_INFO(this->get_logger(), "已更新区域 %d 的多边形，点数: %lu", area_index, points.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "更新区域 %d 失败", area_index);
    }
  } else {
    // tunnel参数，分别写x/y
    int tunnel_index = area_index - 4;
    std::string x_param = parameter_names_[area_index];
    std::string y_param = parameter_names_[area_index + 1];
    std::string x_value = "[ ";
    std::string y_value = "[ ";
    for (size_t i = 0; i < points.size(); ++i) {
      if (i > 0) { x_value += ", "; y_value += ", "; }
      x_value += std::to_string(points[i].x);
      y_value += std::to_string(points[i].y);
    }
    x_value += " ]";
    y_value += " ]";
    // x写入当前index，y写入index+1
    if (modifyConfigFile(config_file_path_, x_param, x_value, area_index)) {
      RCLCPP_INFO(this->get_logger(), "已更新%s: %s", x_param.c_str(), x_value.c_str());
    }
    if (modifyConfigFile(config_file_path_, y_param, y_value, area_index)) {
      RCLCPP_INFO(this->get_logger(), "已更新%s: %s", y_param.c_str(), y_value.c_str());
    }
    // 跳过下一个（y参数），下次自动切到下一个x参数
    current_area_index_ = area_index + 2 - 1; // -1因为handleAutoMode会+1
  }
}

/**
 * @brief 重新加载costmap（预留接口）。
 */
void ConfigDynamicModifier::reloadCostmap()
{
  // 这里可以添加重新加载costmap的逻辑
  // 例如，通过服务调用或重新启动相关节点
  RCLCPP_INFO(this->get_logger(), "请求重新加载costmap");
  
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "已请求重新加载costmap";
  status_pub_->publish(status_msg);
}

}  // namespace config_dynamic_modifier

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<config_dynamic_modifier::ConfigDynamicModifier>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 