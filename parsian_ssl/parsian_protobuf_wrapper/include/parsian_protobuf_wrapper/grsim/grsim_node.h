//
// Created by kian behzad on 3/14/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>

#include "net/udpsend.h"

#include "rclcpp/rclcpp.hpp"
#include "parsian_msgs/msg/parsian_robot_command.hpp"
using std::placeholders::_1;


class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode();

private:
    void topic_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const;
    rclcpp::Subscription<parsian_msgs::msg::ParsianRobotCommand>::SharedPtr subscription_;
};



#endif //PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
