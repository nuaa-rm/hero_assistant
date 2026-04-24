#include "include/robot_behaviors/plugins/keep_away_from_obstacles_action.hpp"


namespace nav2_behavior_tree
{

    KeepAwayFromObstaclesAction::KeepAwayFromObstaclesAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf) : BtActionNode<robot_msgs::action::KeepAwayFromObstacles>(xml_tag_name, action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        getInput("speed", goal_.speed);
        getInput("distance", goal_.distance);
    }

    void KeepAwayFromObstaclesAction::on_tick()
    {
        increment_recovery_count();
        RCLCPP_WARN(node_->get_logger(), "恢复行为树成功调用 speed: %lf, distance: %lf", goal_.speed, goal_.distance);
    }

} // namespace robot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::KeepAwayFromObstaclesAction>(
            name, "keep_away_from_obstacles", config);
    };

    factory.registerBuilder<nav2_behavior_tree::KeepAwayFromObstaclesAction>(
        "KeepAwayFromObstacles", builder);
}
