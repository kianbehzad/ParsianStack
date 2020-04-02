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
#include "parsian_msgs/msg/ssl_vision_geometry.hpp"
#include "parsian_msgs/msg/parsian_robot_command.hpp"
#include "parsian_msgs/msg/parsian_world_model.hpp"
#include "parsian_util/core/knowledge.h"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class WorldModelNode : public rclcpp::Node
{
public:
    WorldModelNode(const rclcpp::NodeOptions & options);

private:
    void vision_detection_callback(const parsian_msgs::msg::SSLVisionDetection::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::SSLVisionDetection>::SharedPtr vision_detection_subscription;

    void vision_geometry_callback(const parsian_msgs::msg::SSLVisionGeometry::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::SSLVisionGeometry>::SharedPtr vision_geometry_subscription;

    void command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::ParsianRobotCommand>::SharedPtr command_subscription[knowledge::MAX_ROBOT_NUM];

    rclcpp::Publisher<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_publisher;



};



#endif //PARSIAN_WORLD_MODEL_WORLDMODEL_NODE_H
