//
// Created by kian behzad on 4/7/20.
//
#include <thread>

#include <QDebug>
#include <QApplication>
#include <QPushButton>

#include "rclcpp/rclcpp.hpp"
#include "parsian_gui/interface/interface_node.h"


struct NodeInThread
{
    NodeInThread(int _argc, char * _argv[]){
        argc = _argc; argv = _argv;
    }
    void operator()(){
        rclcpp::init(argc, argv);
        interface_node = std::make_shared<InterfaceNode>(argc, argv, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
        rclcpp::spin(interface_node);
        rclcpp::shutdown();
    }

    std::shared_ptr<InterfaceNode> interface_node;
    int argc;
    char ** argv;
};

int main(int argc, char * argv[])
{
    // run interface node in a seperate thread
    NodeInThread node_in_thread{argc, argv};
    std::thread node_thread{node_in_thread};
    node_thread.detach();

    //create interface application
    QApplication app{argc, argv};
    QPushButton* button = new QPushButton();
    button->show();
    app.exec();



    return 0;
}
