#include "grsim_node.h"

GrsimNode::GrsimNode() : Node("grsim_node")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&GrsimNode::topic_callback, this, _1));
}

void GrsimNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}
