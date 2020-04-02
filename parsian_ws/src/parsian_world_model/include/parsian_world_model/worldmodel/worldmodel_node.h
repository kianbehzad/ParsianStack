//
// Created by kian behzad on 4/2/20.
//

#ifndef PARSIAN_WORLD_MODEL_WORLDMODEL_NODE_H
#define PARSIAN_WORLD_MODEL_WORLDMODEL_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "parsian_msgs/msg/ssl_vision_detection.hpp"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class WorldModelNode : public rclcpp::Node
{
public:
    WorldModelNode(const rclcpp::NodeOptions & options);

private:
//    void vision_detection_callback(const parsian_msgs::msg::SSLVisionDetection::SharedPtr msg);
//    rclcpp::Subscription<parsian_msgs::msg::SSLVisionDetection>::SharedPtr vision_detection_subscription;


};



#endif //PARSIAN_WORLD_MODEL_WORLDMODEL_NODE_H
