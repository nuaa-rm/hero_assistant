/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
/**
 * @file custom_layer.hpp
 * @brief nav2_costmap_2d 的自定义代价地图层插件头文件
 * @author cmyhj
 * @date 2025-07-11
 *
 * 参考教程：https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 */
#ifndef CUSTOM_LAYER_HPP_
#define CUSTOM_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop


#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace nav2_custom_costmap_plugin
{

/**
 * @class CustomLayer
 * @brief 继承自 nav2_costmap_2d::CostmapLayer 的自定义代价地图层插件。
 *        可通过参数设置移除指定区域（多边形）为 FREE_SPACE。
 */
class CustomLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief 构造函数，初始化边界和移除区域。
   */
  CustomLayer();

  /**
   * @brief 插件初始化函数，声明参数并初始化。
   */
  virtual void onInitialize() override;

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
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  /**
   * @brief 更新代价地图内容。
   * @param master_grid 主代价地图
   * @param min_i 最小i索引
   * @param min_j 最小j索引
   * @param max_i 最大i索引
   * @param max_j 最大j索引
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief 重置层（无操作）。
   */
  virtual void reset() override { return; }

  /**
   * @brief 足迹变化时的回调。
   */
  virtual void onFootprintChanged() override;

  /**
   * @brief 是否可清除（不可清除）。
   * @return false
   */
  virtual bool isClearable() override { return false; }

private:
  // bool makeROIFromString(const std::string & ROI_string,std::vector<geometry_msgs::msg::Point> & ROI);

  double last_min_x_, last_min_y_, last_max_x_, last_max_y_; ///< 上次边界

  // Indicates that the entire custom should be recalculated next time.
  bool need_recalculation_; ///< 是否需要重新计算整个区域


  std::string removed_area_1_; ///< 移除区域1参数
  std::string removed_area_2_; ///< 移除区域2参数
  std::string removed_area_3_; ///< 移除区域3参数
  std::string removed_area_4_; ///< 移除区域4参数

  std::vector<std::vector<geometry_msgs::msg::Point>> removed_area_; ///< 移除区域多边形点集
};

}  // namespace nav2_custom_costmap_plugin

#endif  // CUSTOM_LAYER_HPP_
