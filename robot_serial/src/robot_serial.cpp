#include "serial_pro/robot_serial.h"

void RobotSerial::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    Velocity velocity{
        (float)(msg->linear.x),
        (float)(msg->linear.y),
        (float)(msg->angular.z),
    };
    SOF++;
    sentrySerial.write(0x0501, SOF, velocity);
    RCLCPP_INFO(this->get_logger(), "%f %f %f", msg->linear.x, msg->linear.y, msg->angular.z);
}

void RobotSerial::navigationCommandCallback(const rm_interfaces::msg::NavigationCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received navigation command: %d, target_id: %d", msg->command, msg->target_id);
    
    if (msg->command == 1) {
        recorded_position_x_ = current_robot_x_;
        recorded_position_y_ = current_robot_y_;
        RCLCPP_INFO(this->get_logger(), "Position recorded: (%.2f, %.2f)", recorded_position_x_, recorded_position_y_);
    } else if (msg->command == 2) {
        rm_interfaces::msg::SelfPosition self_pos;
        self_pos.header.stamp = this->now();
        self_pos.x = recorded_position_x_;
        self_pos.y = recorded_position_y_;
        self_pos.z = 0.0;
        SelfPositionPublisher->publish(self_pos);
        RCLCPP_INFO(this->get_logger(), "Going to recorded position: (%.2f, %.2f)", recorded_position_x_, recorded_position_y_);
    }
}

void RobotSerial::updateDistanceInfo()
{
    float dx = target_position_x_ - current_robot_x_;
    float dy = target_position_y_ - current_robot_y_;
    float distance = sqrt(dx * dx + dy * dy);
    
    rm_interfaces::msg::DistanceInfo distance_info;
    distance_info.header.stamp = this->now();
    distance_info.current_x = current_robot_x_;
    distance_info.current_y = current_robot_y_;
    distance_info.target_x = target_position_x_;
    distance_info.target_y = target_position_y_;
    distance_info.distance = distance;
    distance_info.arrived = distance < 0.5;
    
    DistanceInfoPublisher->publish(distance_info);
}

RobotSerial::RobotSerial() : Node("robot_serial_node")
{
    declare_parameter("/serial_name_sentry", "/dev/sentry_serial");
    sentrySerial = std::move(sentry::SentrySerial(get_parameter("/serial_name_sentry").as_string(), 115200));

    dog_cnt_ = 0;
    this->declare_parameter("dog_threshold", 10);
    this->declare_parameter("frequency", 100.0);

    this->get_parameter("dog_threshold", dog_threshold_);
    this->get_parameter("frequency", frequency);

    period_ns = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / frequency));

    current_robot_x_ = 0.0;
    current_robot_y_ = 0.0;
    recorded_position_x_ = 0.0;
    recorded_position_y_ = 0.0;
    target_position_x_ = 0.0;
    target_position_y_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "robot_serial init success");

    sentrySerial.registerCallback(0x0606, [this](const data_received_packag_200hz_t& msg)
    {
        dog_cnt_ = 0;
    });

    sentrySerial.registerCallback(0x0607, [this](const data_received_packag_1hz_t& msg)
    {
        rm_interfaces::msg::Robotp _Robotp;
        _Robotp.x = msg.robot_pos.x;
        _Robotp.y = msg.robot_pos.y;
        _Robotp.angle = msg.robot_pos.angle;
        RobotpPublisher->publish(_Robotp);

        current_robot_x_ = msg.robot_pos.x;
        current_robot_y_ = msg.robot_pos.y;

        dog_cnt_ = 0;
    });

    RobotpPublisher = create_publisher<rm_interfaces::msg::Robotp>("/robot/robotp", 1);
    RobotstatusPublisher = create_publisher<rm_interfaces::msg::Robotstatus>("/robot/robotstatus", 1);
    SelfPositionPublisher = create_publisher<rm_interfaces::msg::SelfPosition>("/SelfPosition", 1);
    DistanceInfoPublisher = create_publisher<rm_interfaces::msg::DistanceInfo>("/robot/distance_info", 1);

    VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1,
                                                                          std::bind(&RobotSerial::velocityCallback,
                                                                              this,
                                                                              std::placeholders::_1));
    NavigationCommandSubscription = create_subscription<rm_interfaces::msg::NavigationCommand>("/robot/navigation_command", 1,
                                                                                              std::bind(&RobotSerial::navigationCommandCallback,
                                                                                                  this,
                                                                                                  std::placeholders::_1));

    timer_ = this->create_wall_timer(period_ns, std::bind(&RobotSerial::timer_callback, this));

    distance_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotSerial::updateDistanceInfo, this));

    sentrySerial.spin(true);
}

void RobotSerial::timer_callback()
{
    dog_cnt_++;
    if (dog_cnt_ > dog_threshold_)
    {
        RCLCPP_ERROR(this->get_logger(), "长时间未收到C板数据，重启串口");
        throw;
    }
}