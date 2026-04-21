/**
 * @file custom_layer.cpp
 * @brief nav2_costmap_2d 的自定义代价地图层插件实现文件
 */
#include "nav2_custom_costmap_plugin/custom_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/version.h"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_custom_costmap_plugin
{

/**
 * @brief 构造函数，初始化边界和移除区域。
 */
CustomLayer::CustomLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
  removed_area_.resize(4);
}

/**
 * @brief 插件初始化函数，声明参数并初始化。
 */
void
CustomLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("removed_area_1", rclcpp::ParameterValue("[]"));
  declareParameter("removed_area_2", rclcpp::ParameterValue("[]"));
  declareParameter("removed_area_3", rclcpp::ParameterValue("[]"));
  declareParameter("removed_area_4", rclcpp::ParameterValue("[]"));
  

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "removed_area_1", removed_area_1_);
  node->get_parameter(name_ + "." + "removed_area_2", removed_area_2_);
  node->get_parameter(name_ + "." + "removed_area_3", removed_area_3_);
  node->get_parameter(name_ + "." + "removed_area_4", removed_area_4_);
 

  if (removed_area_1_ != "" && removed_area_1_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    if (!nav2_costmap_2d::makeFootprintFromString(removed_area_1_, removed_area_[0])) {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"), "The removed_area_1_ parameter is invalid: \"%s\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
        removed_area_1_.c_str());
    }
  }
  if (removed_area_2_ != "" && removed_area_2_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    if (!nav2_costmap_2d::makeFootprintFromString(removed_area_2_, removed_area_[1])) {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"), "The removed_area_2_ parameter is invalid: \"%s\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
        removed_area_2_.c_str());
    }
  }
  if (removed_area_3_ != "" && removed_area_3_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    if (!nav2_costmap_2d::makeFootprintFromString(removed_area_3_, removed_area_[2])) {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"), "The removed_area_3_ parameter is invalid: \"%s\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
        removed_area_3_.c_str());
    }
  }
  if (removed_area_4_ != "" && removed_area_4_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    if (!nav2_costmap_2d::makeFootprintFromString(removed_area_4_, removed_area_[3])) {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"), "The removed_area_4_ parameter is invalid: \"%s\"!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
        removed_area_4_.c_str());
    }
  }

  need_recalculation_ = false;
  current_ = true;
}

/**
 * @brief 更新代价地图边界。
 * @param robot_x 机器人x坐标
 * @param robot_y 机器人y坐标
 * @param robot_yaw 机器人朝向
 * @param min_x 最小x边界
 * @param min_y 最小y边界
 * @param max_x 最大x边界
 * @param max_y 最大y边界
 */
void
CustomLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

/**
 * @brief 足迹变化时的回调。
 */
void
CustomLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "CustomLayer::onFootprintChanged(): 足迹点数量: %lu",
    layered_costmap_->getFootprint().size());
}

/**
 * @brief 更新代价地图内容。
 * @param master_grid 主代价地图
 * @param min_i 最小i索引
 * @param min_j 最小j索引
 * @param max_i 最大i索引
 * @param max_j 最大j索引
 */
void
CustomLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - 是更新窗口的坐标。
  // 这些变量用于仅在该窗口内更新代价地图，避免更新整个区域。
  //
  // 如果必要，使用地图大小固定窗口坐标。
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      costmap_[index] = NO_INFORMATION;
    }
  }
  for (unsigned int i = 0; i < removed_area_.size(); i++) 
  {
    setConvexPolygonCost(removed_area_[i], nav2_costmap_2d::FREE_SPACE);
  }
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  return;
}

}  // namespace nav2_custom_costmap_plugin

// This is the macro allowing a nav2_custom_costmap_plugin::CustomLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_costmap_plugin::CustomLayer, nav2_costmap_2d::Layer)
