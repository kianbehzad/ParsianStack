//
// Created by kian behzad on 3/14/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode();

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};



#endif //PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
