//
// Created by kian behzad on 3/14/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "parsian_msgs/msg/parsian_robot_command.hpp"
#include "parsian_msgs/msg/parsian_world_model.hpp"
#include "parsian_msgs/srv/grsim_ball_replacement.hpp"
#include "parsian_msgs/srv/grsim_robot_replacement.hpp"
#include "parsian_util/core/knowledge.h"
#include "parsian_protobuf_wrapper/common/net/udpsend.h"
#include "parsian_protobuf_wrapper/proto/grSim_Commands.pb.h"
#include "parsian_protobuf_wrapper/proto/grSim_Replacement.pb.h"
#include "parsian_protobuf_wrapper/proto/grSim_Packet.pb.h"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode(const rclcpp::NodeOptions & options);

private:
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    void worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_subscription;

    void command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::ParsianRobotCommand>::SharedPtr command_subscription[knowledge::MAX_ROBOT_NUM];

    void ball_replacement_callback(const std::shared_ptr<parsian_msgs::srv::GrsimBallReplacement::Request> request, std::shared_ptr<parsian_msgs::srv::GrsimBallReplacement::Response> response);
    rclcpp::Service<parsian_msgs::srv::GrsimBallReplacement>::SharedPtr ball_replacement_service;

    void robot_replacement_callback(const std::shared_ptr<parsian_msgs::srv::GrsimRobotReplacement::Request> request, std::shared_ptr<parsian_msgs::srv::GrsimRobotReplacement::Response> response);
    rclcpp::Service<parsian_msgs::srv::GrsimRobotReplacement>::SharedPtr robot_replacement_service;

    UDPSend* udp_send;
    std::string grsim_ip;
    int grsim_command_listen_port;

    bool is_our_color_yellow = false;

    grSim_Commands grsim_commands;
    grSim_Packet grsim_packet;


};



#endif //PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
