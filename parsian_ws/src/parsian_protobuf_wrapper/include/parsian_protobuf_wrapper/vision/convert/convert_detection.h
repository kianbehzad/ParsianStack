#ifndef _COVERT_DETECTION
#define _COVERT_DETECTION

/**
 * Functions to convert SSL vision detection packets to the ROS format.
 * Works for legacy and current SSL vision formats.
 */


#include "parsian_protobuf_wrapper/messages_robocup_ssl_detection.pb.h"
#include "parsian_msgs/ssl_vision_detection.h"

#include "convert_units.h"


namespace pr {
/**
 * Converts an SSL DetectionFrame to the ROS version.
 */
parsian_msgs::ssl_vision_detection convert_detection_frame(SSL_DetectionFrame protoFrame, bool isYellow, bool isLeft);


/**
 * Converts a protoBuf DetectionBall to the ROS version.
 */
parsian_msgs::ssl_vision_detection_ball convert_detection_ball(SSL_DetectionBall protoBall);


/**
 * Converts a protoBuf DetectionRobot to the ROS version.
 */
parsian_msgs::ssl_vision_detection_robot convert_detection_robot(SSL_DetectionRobot protoBot);
}

#endif // _CONVERT_DETECTION