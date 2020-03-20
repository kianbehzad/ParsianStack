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

    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&GrsimNode::worldmodel_callback, this, _1));

    // set up agent_command callbacks
    for(int i{}; i < knowledge::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<parsian_msgs::msg::ParsianRobotCommand>("/agent_" + std::to_string(i) + "/command", 10, std::bind(&GrsimNode::command_callback, this, _1));

    // set up udp connection
    grsim_ip = parameters_client->get_parameter("grsim_ip", grsim_ip);
    grsim_command_listen_port = parameters_client->get_parameter("grsim_command_listen_port", grsim_command_listen_port);
    RCLCPP_INFO(this->get_logger(), "stablish udp com: " + grsim_ip +":%d", grsim_command_listen_port);
    udp_send = new UDPSend(grsim_ip, grsim_command_listen_port);

    // protobuf packets
    grsim_commands = new grSim_Commands;
}

void GrsimNode::command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "robot_command: '%d', '%d'", msg->robot_id, msg->kick_speed);
    grSim_Robot_Command* grsim_robot_command = grsim_commands->add_robot_commands();
    grsim_robot_command->set_id(msg->robot_id);
    grsim_robot_command->set_kickspeedx(msg->kick_speed / 100.0);
    grsim_robot_command->set_kickspeedz(msg->kick_speedz / 200.0);
    grsim_robot_command->set_veltangent(0);
    grsim_robot_command->set_velnormal(0);
    grsim_robot_command->set_velangular(0);
    grsim_robot_command->set_wheel1(msg->wheel1);
    grsim_robot_command->set_wheel2(msg->wheel2);
    grsim_robot_command->set_wheel3(msg->wheel3);
    grsim_robot_command->set_wheel4(msg->wheel4);
    grsim_robot_command->set_spinner(msg->spinner != 0u);
    grsim_robot_command->set_wheelsspeed(msg->wheels_speed != 0u);
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