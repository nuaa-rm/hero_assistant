#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/navigation_command.hpp"
#include "rm_interfaces/msg/distance_info.hpp"
#include "rm_interfaces/msg/robotp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class NavigationManager : public rclcpp::Node
{
public:
    NavigationManager() : Node("navigation_manager")
    {
        this->declare_parameter("target_point.x", 0.0);
        this->declare_parameter("target_point.y", 0.0);
        this->declare_parameter("target_point.z", 0.0);
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("robot_base_frame", "base_link");
        
        this->get_parameter("target_point.x", target_position_x_);
        this->get_parameter("target_point.y", target_position_y_);
        this->get_parameter("target_point.z", target_position_z_);
        this->get_parameter("global_frame", global_frame_);
        this->get_parameter("robot_base_frame", robot_base_frame_);

        recorded_position_x_ = 0.0;
        recorded_position_y_ = 0.0;
        recorded_position_z_ = 0.0;//局内标记点
        current_robot_x_ = 0.0;
        current_robot_y_ = 0.0;
        current_robot_z_ = 0.0;//当前坐标
        has_recorded_position_ = false;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        navigation_command_sub_ = this->create_subscription<rm_interfaces::msg::NavigationCommand>(
            "/robot/navigation_command",//下位机传上来的命令
            10,
            std::bind(&NavigationManager::navigationCommandCallback, this, std::placeholders::_1));

        robot_position_sub_ = this->create_subscription<rm_interfaces::msg::Robotp>(
            "/robot/robotp",//裁判系统传上来的机器人位置
            10,
            std::bind(&NavigationManager::robotPositionCallback, this, std::placeholders::_1));

        distance_info_pub_ = this->create_publisher<rm_interfaces::msg::DistanceInfo>(
            "/robot/distance_info",//发布与目标点的距离至下位机
            10);

        client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this,
            "navigate_to_pose");

        distance_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&NavigationManager::updateDistanceInfo, this));

        RCLCPP_INFO(this->get_logger(), "Navigation manager initialized");
        RCLCPP_INFO(this->get_logger(), "Target point: (%.2f, %.2f, %.2f)", 
                    target_position_x_, target_position_y_, target_position_z_);
    }

private:
    /*
     * @brief 处理下位机传上来的导航命令
     * 
     * @param msg 下位机传上来的导航命令
     */
    void navigationCommandCallback(const rm_interfaces::msg::NavigationCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received navigation command: %d", msg->command);

        if (msg->command == 1)//记录当前位置
        {
            recorded_position_x_ = current_robot_x_;
            recorded_position_y_ = current_robot_y_;
            recorded_position_z_ = current_robot_z_;
            has_recorded_position_ = true;
            RCLCPP_INFO(this->get_logger(), "Position recorded: (%.2f, %.2f, %.2f)", 
                        recorded_position_x_, recorded_position_y_, recorded_position_z_);
        }
        else if (msg->command == 2)//导航到记录位置
        {
            if (has_recorded_position_)
            {
                sendNavigationGoal(recorded_position_x_, recorded_position_y_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No position recorded yet!");
            }
        }
        else if (msg->command == 3)//导航到赛前记录目标位置（一般为敌方装甲板位置）
        {
            sendNavigationGoal(target_position_x_, target_position_y_);
        }
    }

    /*
     * @brief 裁判系统传上来的机器人位置，暂时没有使用
     * 
     * @param msg 裁判系统传上来的机器人位置
     */
    void robotPositionCallback(const rm_interfaces::msg::Robotp::SharedPtr msg)
    {
        // current_robot_x_ = msg->x;
        // current_robot_y_ = msg->y;
    }

    /*
     * @brief 从TFtree获取机器人机器人位置robot_base_frame,更新current_robot_
     * 
     * @param global_frame 全局坐标系
     * @param robot_base_frame 机器人机器人坐标系
     */
    void getRobotPositionFromTF()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                global_frame_, 
                robot_base_frame_, 
                tf2::TimePointZero);

            current_robot_x_ = transform_stamped.transform.translation.x;
            current_robot_y_ = transform_stamped.transform.translation.y;
            current_robot_z_ = transform_stamped.transform.translation.z;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
        }
    }

    /*
     * @brief 发送导航目标点TODO:未测试
     * 
     * @param x 目标点x坐标
     * @param y 目标点y坐标
     */
    void sendNavigationGoal(double x, double y)
    {
        if (!client_ptr_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending navigation goal to: (%.2f, %.2f)", x, y);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigationManager::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigationManager::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigationManager::resultCallback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        (void)feedback;
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation completed successfully");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }
    /*
     * @brief 更新距离信息TODO:未测试
     * 
     */
    void updateDistanceInfo()//更新距离信息
    {
        getRobotPositionFromTF();

        float dx = target_position_x_ - current_robot_x_;
        float dy = target_position_y_ - current_robot_y_;
        float dz = target_position_z_ - current_robot_z_;
        float distance = sqrt(dx * dx + dy * dy);

        rm_interfaces::msg::DistanceInfo distance_info;
        // distance_info.header.stamp = this->now();
        // distance_info.current_x = current_robot_x_;
        // distance_info.current_y = current_robot_y_;
        // distance_info.current_z = current_robot_z_;
        // distance_info.target_x = target_position_x_;
        // distance_info.target_y = target_position_y_;
        // distance_info.target_z = target_position_z_;
        distance_info.distance = distance;
        // distance_info.height_diff = dz;
        // distance_info.arrived = distance < 0.5;

        distance_info_pub_->publish(distance_info);
    }

    rclcpp::Subscription<rm_interfaces::msg::NavigationCommand>::SharedPtr navigation_command_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Robotp>::SharedPtr robot_position_sub_;
    rclcpp::Publisher<rm_interfaces::msg::DistanceInfo>::SharedPtr distance_info_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr distance_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double target_position_x_;
    double target_position_y_;
    double target_position_z_;
    double recorded_position_x_;
    double recorded_position_y_;
    double recorded_position_z_;
    double current_robot_x_;
    double current_robot_y_;
    double current_robot_z_;
    bool has_recorded_position_;
    std::string global_frame_;
    std::string robot_base_frame_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationManager>());
    rclcpp::shutdown();
    return 0;
}