#ifndef ROBOT_SERIAL_ROBOT_SERIAL_H
#define ROBOT_SERIAL_ROBOT_SERIAL_H

#include <iomanip>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sentry_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_sentry.h"

class RobotSerial : public rclcpp::Node {
private:
    sentry::SentrySerial sentrySerial;
    rclcpp::Clock rosClock;

    int dog_cnt_;
    int dog_threshold_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr distance_timer_;
    std::chrono::milliseconds period_ns;
    double frequency;

    float current_robot_x_;
    float current_robot_y_;
    float recorded_position_x_;
    float recorded_position_y_;
    float target_position_x_;
    float target_position_y_;

    rclcpp::Publisher<rm_interfaces::msg::Robotp>::SharedPtr RobotpPublisher;
    rclcpp::Publisher<rm_interfaces::msg::Robotstatus>::SharedPtr RobotstatusPublisher;
    rclcpp::Publisher<rm_interfaces::msg::SelfPosition>::SharedPtr SelfPositionPublisher;

    
    rclcpp::Subscription<rm_interfaces::msg::DistanceInfo>::SharedPtr DistanceInfoSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelocitySubscription;
    rclcpp::Subscription<rm_interfaces::msg::NavigationCommand>::SharedPtr NavigationCommandSubscription;

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void navigationCommandCallback(const rm_interfaces::msg::NavigationCommand::SharedPtr msg);
    void distanceInfoCallback(const rm_interfaces::msg::DistanceInfo::SharedPtr msg);
    void timer_callback();

public:
    explicit RobotSerial();
};

#endif //ROBOT_SERIAL_ROBOT_SERIAL_H