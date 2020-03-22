/**
 * Functions to convert SSL vision detection packets to the ROS format.
 * Works for legacy and current SSL vision formats.
 */

#include <parsian_msgs/ssl_vision_detection.h>
#include <parsian_protobuf_wrapper/messages_robocup_ssl_detection.pb.h>
#include <parsian_protobuf_wrapper/ssl-vision/convert/convert_units.h>
#include "parsian_protobuf_wrapper/ssl-vision/convert/convert_detection.h"

namespace pr {

/**
 * Converts a protoBuf ssl_vision_detection to the ROS version.
 */
parsian_msgs::ssl_vision_detection convert_detection_frame(SSL_DetectionFrame protoFrame, bool isYellow, bool isLeft) {
    parsian_msgs::ssl_vision_detection rosFrame;

    rosFrame.frame_number = protoFrame.frame_number();
    rosFrame.t_capture = protoFrame.t_capture();
    rosFrame.t_sent = protoFrame.t_sent();
    rosFrame.camera_id = protoFrame.camera_id();

    for (int i = 0; i < protoFrame.balls().size(); ++i) {
        SSL_DetectionBall protoBall = protoFrame.balls().Get(i);
        parsian_msgs::ssl_vision_detection_ball rosBall = convert_detection_ball(protoBall);

        rosFrame.balls.push_back(rosBall);
    }

    for (int i = 0; i < protoFrame.robots_yellow().size(); ++i) {
        SSL_DetectionRobot protoBot = protoFrame.robots_yellow().Get(i);
        parsian_msgs::ssl_vision_detection_robot rosBot = convert_detection_robot(protoBot);
        if (isYellow) {
            rosFrame.us.push_back(rosBot);
        } else {
            rosFrame.them.push_back(rosBot);
        }
    }

    for (int i = 0; i < protoFrame.robots_blue().size(); ++i) {
        SSL_DetectionRobot protoBot = protoFrame.robots_blue().Get(i);
        parsian_msgs::ssl_vision_detection_robot rosBot = convert_detection_robot(protoBot);
        if (isYellow) {
            rosFrame.them.push_back(rosBot);
        } else {
            rosFrame.us.push_back(rosBot);
        }
    }

    return rosFrame;
}


/**
 * Converts a protoBuf DetectionBall to the ROS version.
 */
parsian_msgs::ssl_vision_detection_ball convert_detection_ball(SSL_DetectionBall protoBall) {
    parsian_msgs::ssl_vision_detection_ball rosBall;

    rosBall.confidence = protoBall.confidence();
    rosBall.area = protoBall.area();
    rosBall.pos.x = mm_to_m(protoBall.x());
    rosBall.pos.y = mm_to_m(protoBall.y());
    rosBall.z = mm_to_m(protoBall.z());
    rosBall.pixel_pos.x = protoBall.pixel_x();
    rosBall.pixel_pos.x = protoBall.pixel_y();

    return rosBall;
}


/**
 * Converts a protoBuf DetectionRobot to the ROS version.
 */
parsian_msgs::ssl_vision_detection_robot convert_detection_robot(SSL_DetectionRobot protoBot) {
    parsian_msgs::ssl_vision_detection_robot rosBot;

    rosBot.confidence = protoBot.confidence();
    rosBot.robot_id = protoBot.robot_id();
    rosBot.pos.x = mm_to_m(protoBot.x());
    rosBot.pos.y = mm_to_m(protoBot.y());
    rosBot.orientation = protoBot.orientation();
    rosBot.pixel_pos.x = protoBot.pixel_x();
    rosBot.pixel_pos.y = protoBot.pixel_y();
    rosBot.height = mm_to_m(protoBot.height());

    return rosBot;
}

}
