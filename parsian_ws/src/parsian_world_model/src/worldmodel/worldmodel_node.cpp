#include "parsian_world_model/worldmodel/worldmodel_node.h"

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions & options) : Node("worldmodel_node", options)
{


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