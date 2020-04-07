//
// Created by kian behzad on 4/7/20.
//
#include "rclcpp/rclcpp.hpp"
#include "parsian_gui/interface/interface_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InterfaceNode>(argc, argv, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}
