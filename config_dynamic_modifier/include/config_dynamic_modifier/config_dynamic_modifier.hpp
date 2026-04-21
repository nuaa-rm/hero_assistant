#ifndef CONFIG_DYNAMIC_MODIFIER_HPP_
#define CONFIG_DYNAMIC_MODIFIER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @file config_dynamic_modifier.hpp
 * @brief 配置动态修改器节点头文件，声明自动模式下的多边形区域收集与配置文件动态写入相关接口。
 */

namespace config_dynamic_modifier
{

/**
 * @class ConfigDynamicModifier
 * @brief ROS2节点，实现自动收集多边形区域并动态修改配置文件。
 */
class ConfigDynamicModifier : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化节点、参数、订阅者和发布者。
     */
    ConfigDynamicModifier();

private:
    /**
     * @brief 点话题回调函数，处理自动模式下的点收集。
     * @param msg 点消息
     */
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    /**
     * @brief 命令话题回调函数（预留）。
     * @param msg 命令消息
     */
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    /**
     * @brief 修改yaml配置文件中指定参数的值。
     * @param config_file_path 配置文件路径
     * @param parameter_name 参数名
     * @param new_value 新值
     * @param area_index 区域索引
     * @return 是否修改成功
     */
    bool modifyConfigFile(const std::string& config_file_path, 
                         const std::string& parameter_name, 
                         const std::string& new_value,
                         const int area_index);
    /**
     * @brief 向当前多边形添加一个点。
     * @param point 新增的点
     */
    void addPointToPolygon(const geometry_msgs::msg::Point& point);
    /**
     * @brief 清空当前多边形点。
     */
    void clearCurrentPolygon();
    /**
     * @brief 完成当前多边形的收集，写入配置并切换区域。
     */
    void finishPolygon();
    /**
     * @brief 根据点生成多边形字符串（用于yaml写入）。
     * @param points 点列表
     * @return 多边形字符串
     */
    std::string generatePolygonStringFromPoints(const std::vector<geometry_msgs::msg::Point>& points);
    /**
     * @brief 自动模式下处理点的逻辑，收集多边形并自动切换区域。
     * @param point 新增的点
     */
    void handleAutoMode(const geometry_msgs::msg::Point& point);
    /**
     * @brief 切换到多边形收集模式（预留）。
     */
    void switchToPolygonMode();
    /**
     * @brief 切换到单点模式（预留）。
     */
    void switchToSinglePointMode();
    /**
     * @brief 解析多边形字符串为点对vector。
     * @param polygon_str 多边形字符串
     * @return 点对vector
     */
    std::vector<std::pair<double, double>> parsePolygonString(const std::string& polygon_str);
    /**
     * @brief 将点对vector转为字符串。
     * @param polygon 点对vector
     * @return 字符串
     */
    std::string polygonToString(const std::vector<std::pair<double, double>>& polygon);
    /**
     * @brief 更新costmap相关参数并写入配置文件。
     * @param points 点列表
     * @param area_index 区域索引
     */
    void updateCostmapParameters(const std::vector<geometry_msgs::msg::Point>& points, int area_index);
    /**
     * @brief 重新加载costmap（预留接口）。
     */
    void reloadCostmap();

    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_; ///< 点消息订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_; ///< 命令消息订阅者

    // 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_; ///< 状态消息发布者

    // 参数
    std::string config_file_path_; ///< 配置文件路径
    std::string point_topic_;      ///< 点话题名
    std::string command_topic_;    ///< 命令话题名
    std::string status_topic_;     ///< 状态话题名

    int current_area_index_; ///< 当前选中的区域索引

    // 多边形收集模式
    bool polygon_collection_mode_; ///< 是否为多边形收集模式
    std::vector<geometry_msgs::msg::Point> current_polygon_points_; ///< 当前多边形点
    int min_polygon_points_; ///< 多边形最小点数

    // 自动模式设置
    bool auto_mode_; ///< 是否为自动模式
    int auto_polygon_points_; ///< 自动模式下每个多边形点数
    int click_count_; ///< 当前点击次数
    std::chrono::steady_clock::time_point last_click_time_; ///< 上次点击时间
    double auto_switch_delay_; ///< 自动切换延迟

    // 新增：每个区域的点击次数
    std::vector<long int> auto_polygon_points_list_; ///< 各区域自动模式点数

    std::vector<long int> parameter_points_count_; ///< 各参数所需点数
    std::vector<std::vector<std::pair<double, double>>> current_polygons_; ///< 当前多边形数据
    std::vector<std::string> parameter_names_; ///< 参数名称映射

    // 动态参数回调相关声明
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_param_callback_handle_;
    /**
     * @brief 动态参数变更回调。
     * @param parameters 参数列表
     * @return 设置结果
     */
    rcl_interfaces::msg::SetParametersResult onDynamicParamsChange(const std::vector<rclcpp::Parameter> &parameters);
};

}  // namespace config_dynamic_modifier

#endif  // CONFIG_DYNAMIC_MODIFIER_HPP_ 