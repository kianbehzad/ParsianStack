//
// Created by kian behzad on 3/14/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>

#include "parsian_msgs/msg/parsian_robot_command.hpp"
#include "parsian_msgs/msg/parsian_world_model.hpp"
#include "parsian_util/knowledge/general.h"
#include <string>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;


class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode();

private:
    void worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg) const;
    void command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const;
    rclcpp::Subscription<parsian_msgs::msg::ParsianRobotCommand>::SharedPtr command_subscription[knowledge::Robot::MAX_ROBOT_NUM];
    rclcpp::Subscription<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_subscription;

};



#endif //PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
