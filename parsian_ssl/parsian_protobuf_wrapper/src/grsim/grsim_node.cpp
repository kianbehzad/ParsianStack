#include "grsim_node.h"

GrsimNode::GrsimNode() : Node("grsim_node")
{
    subscription_ = this->create_subscription<parsian_msgs::msg::ParsianRobotCommand>(
            "topic", 10, std::bind(&GrsimNode::topic_callback, this, _1));
}

void GrsimNode::topic_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->kick_speed);
}
