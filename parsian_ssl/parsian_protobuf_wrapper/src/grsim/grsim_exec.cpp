//
// Created by kian behzad on 3/14/20.
//
#include "rclcpp/rclcpp.hpp"
#include "grsim_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GrsimNode>());
    rclcpp::shutdown();
    return 0;
}
