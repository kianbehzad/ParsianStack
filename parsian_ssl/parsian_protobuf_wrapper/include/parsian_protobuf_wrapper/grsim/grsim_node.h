//
// Created by kian behzad on 3/14/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>

#include "parsian_protobuf_wrapper/common/net/udpsend.h"
#include "parsian_msgs/msg/parsian_robot_command.hpp"
#include "parsian_util/general.h"

#include "rclcpp/rclcpp.hpp"
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
