//
// Created by kian behzad on 4/7/20.
//

#ifndef PARSIAN_GUI_INTERFACE_NODE_H
#define PARSIAN_GUI_INTERFACE_NODE_H


#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <vector>
#include <thread>

#include <QDebug>
#include <QString>

#include "parsian_gui/interface/application/aplication_thread.h"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class InterfaceNode : public rclcpp::Node
{
public:
    InterfaceNode(int argc, char * argv[], const rclcpp::NodeOptions & options);

private:
    AplicationThread* aplication_thread;


};



#endif //PARSIAN_GUI_INTERFACE_NODE_H
