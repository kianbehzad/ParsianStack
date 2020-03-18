#include "parsian_protobuf_wrapper/grsim/grsim_node.h"

GrsimNode::GrsimNode() : Node("grsim_node")
{
    for(int i{}; i < knowledge::Robot::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<parsian_msgs::msg::ParsianRobotCommand>("/agent_" + std::to_string(i) + "/command", 10, std::bind(&GrsimNode::command_callback, this, _1));
}

void GrsimNode::command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%d', '%d'", msg->kick_speed, knowledge::Robot::MAX_ROBOT_NUM);
}
