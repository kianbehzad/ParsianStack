#include "parsian_world_model/worldmodel/worldmodel_node.h"


// extern value definitions
WorldModelConfig m_config;
bool isSimulation;
bool isOurSideLeft;
bool isOurColorYellow;


WorldModelNode::WorldModelNode(const rclcpp::NodeOptions & options) : Node("worldmodel_node", options)
{

    // set up parameter client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "parameter service not available, waiting again...");
    }

    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    //get initial parameters
    m_config.camn_num = parameters_client->get_parameter("cam_num", m_config.camn_num);
    m_config.camera_1 = parameters_client->get_parameter("camera_0", m_config.camera_1);
    m_config.camera_2 = parameters_client->get_parameter("camera_0", m_config.camera_2);
    m_config.camera_3 = parameters_client->get_parameter("camera_0", m_config.camera_3);
    m_config.camera_4 = parameters_client->get_parameter("camera_0", m_config.camera_4);
    m_config.camera_5 = parameters_client->get_parameter("camera_0", m_config.camera_5);
    m_config.camera_6 = parameters_client->get_parameter("camera_0", m_config.camera_6);
    m_config.camera_7 = parameters_client->get_parameter("camera_0", m_config.camera_7);
    isSimulation = parameters_client->get_parameter("is_simulation", isSimulation);
    isOurSideLeft = parameters_client->get_parameter("is_our_side_left", isOurSideLeft);
    isOurColorYellow = parameters_client->get_parameter("is_our_color_yellow", isOurColorYellow);

    // set up vision_detection callback
    vision_detection_subscription = this->create_subscription<parsian_msgs::msg::SSLVisionDetection>("/vision_detection", 8, std::bind(&WorldModelNode::vision_detection_callback, this, _1));

    // set up vision_detection callback
    vision_geometry_subscription = this->create_subscription<parsian_msgs::msg::SSLVisionGeometry>("/vision_geom", 8, std::bind(&WorldModelNode::vision_geometry_callback, this, _1));

    // set up agent_command callbacks
    for(int i{}; i < knowledge::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<parsian_msgs::msg::ParsianRobotCommand>("/agent_" + std::to_string(i) + "/command", 3, std::bind(&WorldModelNode::command_callback, this, _1));

}

void WorldModelNode::vision_detection_callback(const parsian_msgs::msg::SSLVisionDetection::SharedPtr msg)
{

}

void WorldModelNode::vision_geometry_callback(const parsian_msgs::msg::SSLVisionGeometry::SharedPtr msg)
{

}

void WorldModelNode::command_callback(const parsian_msgs::msg::ParsianRobotCommand::SharedPtr msg)
{

}

void WorldModelNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "is_simulation")
            {
                isSimulation = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "is_our_side_left")
            {
                isOurSideLeft = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "is_our_color_yellow")
            {
                isOurColorYellow = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "cam_num")
            {
                m_config.camn_num = changed_parameter.value.integer_value;
            }
            else if(changed_parameter.name == "camera_0")
            {
                m_config.camera_0 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_1")
            {
                m_config.camera_1 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_2")
            {
                m_config.camera_2 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_3")
            {
                m_config.camera_3 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_4")
            {
                m_config.camera_4 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_5")
            {
                m_config.camera_5 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_6")
            {
                m_config.camera_6 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_7")
            {
                m_config.camera_7 = changed_parameter.value.bool_value;
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}
