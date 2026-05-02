#ifndef KEEP_AWAY_FROM_OBSTACLES_ACTION_HPP_
#define KEEP_AWAY_FROM_OBSTACLES_ACTION_HPP_

#include <nav2_behavior_tree/bt_action_node.hpp>
#include "robot_msgs/action/keep_away_from_obstacles.hpp"

namespace nav2_behavior_tree
{

    class KeepAwayFromObstaclesAction : public BtActionNode<robot_msgs::action::KeepAwayFromObstacles>
    {
    public:
        KeepAwayFromObstaclesAction(
            const std::string &xml_tag_name,
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        // Override the virtual methods from BtActionNode

        void on_tick() override;
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
                {
                    BT::InputPort<double>("speed", 1.0, "Target speed of the robot"),
                    BT::InputPort<double>("distance", 0.5, "Travel distance"),
                });
        }
    private:
        rclcpp::Node::SharedPtr node_;
    };

} // namespace nav2_behavior_tree

#endif // KEEP_AWAY_FROM_OBSTACLES_ACTION_HPP_