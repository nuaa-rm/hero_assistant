/**
 * @file sentry_strategy_service.hpp
 * @brief 哨兵策略服务头文件
 * @details 定义了哨兵机器人的目标选择策略服务类，包括自瞄信息处理、全向感知融合和目标优先级判断
 * @author elsa
 * @date 2025-01-14
 * @version 1.0
 */

//
// Created by elsa on 25-5-14.
//

#ifndef SENTRY_STRATEGY_SERVICE_HPP
#define SENTRY_STRATEGY_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/srv/enemy_strategist.hpp"
#include "rm_interfaces/msg/enemy_position.hpp"
#include "rm_interfaces/msg/enemy_positions.hpp"
#include "rm_interfaces/msg/autoaim_strategy.hpp"
#include "rm_interfaces/msg/enemy_chasing.hpp"

/**
 * @brief 哨兵策略服务类
 * @details 实现哨兵机器人的智能目标选择策略，通过融合自瞄和全向感知信息，
 * 根据敌人血量、位置和优先级进行目标选择决策
 * 
 * 目标选择策略优先级：
 * 1. 血量少于100的敌人（优先追击）
 * 2. 英雄机器人
 * 3. 前哨站
 * 4. 基地
 * 5. 默认听自瞄系统
 */
class SentryStrategyService : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @details 初始化ROS2节点，创建订阅者和服务，设置消息存储容器
     */
    explicit SentryStrategyService();

private:
    /**
     * @brief 哨兵策略服务回调函数
     * @param request 服务请求，包含所有敌人信息
     * @param response 服务响应，返回选择的目标信息
     * @details 实现哨兵机器人的目标选择策略：
     * 1. 验证输入数据有效性
     * 2. 融合自瞄和全向感知信息
     * 3. 根据血量优先级选择目标
     * 4. 返回选择的目标信息
     * 
     * 目标选择逻辑：
     * - 在主相机和全向感知识别到的范围内，先打血量最少的
     * - 打完后优先打英雄，其余情况听自瞄的
     * - 如果在追击英雄状态就锁英雄做预瞄(暂不做)
     * - 如果在开团进攻状态：英雄>步兵>哨兵>基地>工程
     * - 日常巡逻状态：英雄>步兵>工程>哨兵>前哨站
     * - 对于hp,invincible这类数据优先从blackboard中获取
     * - 自瞄发来的信息包括5辆车、前哨站和基地，决策中需要决定先打谁并发给自瞄
     * - id依次为1,2,3,4,sentry,outpost,base
     */
    void SentryStrategyServiceCallback(const std::shared_ptr<rm_interfaces::srv::EnemyStrategist::Request> request,
                                   std::shared_ptr<rm_interfaces::srv::EnemyStrategist::Response> response);
    
    /**
     * @brief 自瞄策略信息回调函数
     * @param msg 自瞄策略消息
     * @details 接收从行为树中转发来的白名单，是否无敌，血量的信息
     */
    void AutoaimStrategyCallback(rm_interfaces::msg::AutoaimStrategy::SharedPtr msg);
    
    /**
     * @brief 全向感知信息回调函数
     * @param msg 全向感知消息
     * @details 接收全向感知发来的信息，用于补充自瞄检测不到的敌人信息
     */
    void OcclusionCallback(rm_interfaces::msg::EnemyPositions::SharedPtr msg);

    /**
     * @brief 计算偏航角
     * @return 计算得到的偏航角
     * @details 计算当前机器人朝向的偏航角
     */
    double calculate_yaw();

    /**
     * @brief 将整数ID转换为字符串ID
     * @param int_id 整数ID (0-6)
     * @return 对应的字符串ID
     * @details 0->"1", 1->"2", 2->"3", 3->"4", 4->"sentry", 5->"outpost", 6->"base"
     */
    std::string int_id2string_id(int int_id);
    
    /**
     * @brief 将字符串ID转换为整数ID
     * @param string_id 字符串ID
     * @return 对应的整数ID，无效时返回-1
     * @details "1"->0, "2"->1, "3"->2, "4"->3, "sentry"->4, "outpost"->5, "base"->6
     */
    int string_id2int_id(std::string string_id);

    // ROS2通信相关成员变量
    /// 哨兵策略服务
    rclcpp::Service<rm_interfaces::srv::EnemyStrategist>::SharedPtr sentry_strategy_service;
    /// 自瞄策略信息订阅者
    rclcpp::Subscription<rm_interfaces::msg::AutoaimStrategy>::SharedPtr autoaim_strategy_sub;
    /// 全向感知信息订阅者
    rclcpp::Subscription<rm_interfaces::msg::EnemyPositions>::SharedPtr occlusion_detector_sub;
    /// 发布追击信息到决策
    rclcpp::Publisher<rm_interfaces::msg::EnemyChasing>::SharedPtr enemy_chasing_pub;

    // 数据存储相关成员变量
    /// 存储所有从自瞄获取的enemy的信息
    rm_interfaces::msg::EnemyPositions::SharedPtr enemies;
    /// 从决策发来的信息(白名单，血量，是否无敌)
    rm_interfaces::msg::AutoaimStrategy::SharedPtr received_autoaim_strategy;
    /// 存储所有从全向感知相机获取的信息
    rm_interfaces::msg::EnemyPositions::SharedPtr occlusion_detector;

    /// 是否为蓝方队伍标识
    bool is_we_are_blue;

    /// 策略5是否默认追击
    bool if_default_chasing;

    // 目标选择相关成员变量
    /* 0-6依次为英雄、工程、步兵3、步兵4、哨兵、前哨站、基地 */
    /// 是否要锁，0不锁1锁
    int is_lock[7];
    /// 选择要打的敌方机器人血量
    int enemy_blood_to_select[7];
    /// 目标的id序号，0-6依次为英雄、工程、步兵3、步兵4、哨兵、前哨站、基地
    int id_index;
    /// 敌方数量，默认为7
    int enemy_size;

    int dog_cnt;
    int dog_cnt_threshold;

    /// 选车优先级排序，从yaml文件中读取
    std::vector<std::string> aim_priority;

    /// 丢失目标延时，丢失0.5s后才认为是真的丢失目标
    int lost_delay_cnt;
    /// 是否在丢失等待中（只针对全向感知）
    bool if_in_lost_waiting;
    /// 当前全向感知是否检测到
    bool omni_detected;
    /// 当前全向感知检测到的id对应的index
    int omni_detected_id_index;
};

#endif //SENTRY_STRATEGY_SERVICE_HPP
