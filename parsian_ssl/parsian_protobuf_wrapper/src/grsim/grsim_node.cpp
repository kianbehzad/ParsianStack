#include "parsian_protobuf_wrapper/grsim/grsim_node.h"

GrsimNode::GrsimNode() : Node("grsim_node")
{
    worldmodel_subscription = this->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&GrsimNode::worldmodel_callback, this, _1));
    for(int i{}; i < knowledge::Robot::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<parsian_msgs::msg::ParsianRobotCommand>("/agent_" + std::to_string(i) + "/command", 10, std::bind(&GrsimNode::command_callback, this, _1));
}

void GrsimNode::command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "robot_command: '%d', '%d'", msg->robot_id, msg->kick_speed);
}

void GrsimNode::worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "world_model: '%f', '%f'", msg->ball.pos.x, msg->ball.pos.y);
}