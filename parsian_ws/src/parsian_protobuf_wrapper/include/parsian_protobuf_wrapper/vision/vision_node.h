//
// Created by kian behzad on 3/22/20.
//

#ifndef PARSIAN_PROTOBUF_WRAPPER_VISION_NODE_H
#define PARSIAN_PROTOBUF_WRAPPER_VISION_NODE_H


#include "parsian_protobuf_wrapper/common/net/robocup_ssl_client.h"
#include "parsian_protobuf_wrapper/proto/messages_robocup_ssl_wrapper.pb.h"

#include "rclcpp/rclcpp.hpp"



class VisionNode : public rclcpp::Node
{
public:
    VisionNode(const rclcpp::NodeOptions & options);

private:
    RoboCupSSLClient* vision;
    SSL_WrapperPacket vision_packet;
    bool is_our_color_yellow = false;
    bool is_our_side_left = false;


};





#endif //PARSIAN_PROTOBUF_WRAPPER_VISION_NODE_H
