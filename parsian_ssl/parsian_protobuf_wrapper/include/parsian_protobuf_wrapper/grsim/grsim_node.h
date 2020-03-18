//
// Created by kian behzad on 3/14/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>

#include "parsian_msgs/msg/parsian_robot_command.hpp"
#include "parsian_util/knowledge/general.h"
#include <string>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;


class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode();

private:
    void command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const;
    rclcpp::Subscription<parsian_msgs::msg::ParsianRobotCommand>::SharedPtr command_subscription[knowledge::Robot::MAX_ROBOT_NUM];

};



#endif //PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
