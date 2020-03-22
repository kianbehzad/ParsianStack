//
// Created by kian behzad on 3/22/20.
//

#include "parsian_protobuf_wrapper/vision/vision_node.h"


VisionNode::VisionNode(const rclcpp::NodeOptions & options) : Node("vision_node", options)
{
    RCLCPP_INFO(this->get_logger(), "hello from vision_node");
}
