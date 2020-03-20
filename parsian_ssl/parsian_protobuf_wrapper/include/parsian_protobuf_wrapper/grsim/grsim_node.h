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
#include "parsian_util/core/knowledge.h"
#include "parsian_protobuf_wrapper/common/net/udpsend.h"
#include "parsian_protobuf_wrapper/proto/grSim_Commands.pb.h"
#include "parsian_protobuf_wrapper/proto/grSim_Packet.pb.h"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;


class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode(const rclcpp::NodeOptions & options);

private:
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    void worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg) const;
    rclcpp::Subscription<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_subscription;

    void command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const;
    rclcpp::Subscription<parsian_msgs::msg::ParsianRobotCommand>::SharedPtr command_subscription[knowledge::MAX_ROBOT_NUM];

    UDPSend* udp_send;
    std::string grsim_ip;
    int grsim_command_listen_port;

    grSim_Commands* grsim_commands;
    grSim_Packet grsim_packet;


};



#endif //PARSIAN_PROTOBUF_WRAPPER_GRSIM_NODE_H
