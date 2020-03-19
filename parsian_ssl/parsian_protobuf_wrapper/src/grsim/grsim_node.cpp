#include "parsian_protobuf_wrapper/grsim/grsim_node.h"

GrsimNode::GrsimNode(const rclcpp::NodeOptions & options) : Node("grsim_node", options)
{
    // set up parameter client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // set up parameter change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&GrsimNode::worldmodel_callback, this, _1));

    // set up agent_command callbacks
    for(int i{}; i < knowledge::Robot::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<parsian_msgs::msg::ParsianRobotCommand>("/agent_" + std::to_string(i) + "/command", 10, std::bind(&GrsimNode::command_callback, this, _1));

    // set up udp connection
    grsim_ip = parameters_client->get_parameter("grsim_ip", grsim_ip);
    grsim_command_listen_port = parameters_client->get_parameter("grsim_command_listen_port", grsim_command_listen_port);
    RCLCPP_INFO(this->get_logger(), "stablish udp com: " + grsim_ip +":%d", grsim_command_listen_port);
    udp_send = new UDPSend(grsim_ip, grsim_command_listen_port);
}

void GrsimNode::command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "robot_command: '%d', '%d'", msg->robot_id, msg->kick_speed);
}

void GrsimNode::worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "world_model: '%f', '%f'", msg->ball.pos.x, msg->ball.pos.y);
}


void GrsimNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "grsim_ip")
            {
                grsim_ip = changed_parameter.value.string_value;
                udp_send->setIP(grsim_ip);
            }
            else if(changed_parameter.name == "grsim_command_listen_port")
            {
                grsim_command_listen_port = changed_parameter.value.integer_value;
                udp_send->setport(grsim_command_listen_port);
            }

            RCLCPP_INFO(this->get_logger(), "stablish udp com: " + grsim_ip +":%d", grsim_command_listen_port);
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}