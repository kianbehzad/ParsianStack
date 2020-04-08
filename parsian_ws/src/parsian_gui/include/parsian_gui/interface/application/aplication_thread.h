//
// Created by kian behzad on 4/8/20.
//

#ifndef PARSIAN_GUI_APLICATION_THREAD_H
#define PARSIAN_GUI_APLICATION_THREAD_H

#include <QDebug>
#include <thread>

#include "parsian_msgs/msg/parsian_world_model.hpp"

#include "rclcpp/rclcpp.hpp"



class InterfaceNode; //forward declaration


class AplicationThread
{
public:
    AplicationThread(int argc, char * argv[], InterfaceNode* node);

    void operator()();

private:
    InterfaceNode *interface_node;

    void worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_subscription;


};


#endif //PARSIAN_GUI_APLICATION_THREAD_H
