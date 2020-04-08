//
// Created by kian behzad on 4/7/20.
//
#include <thread>

#include <QDebug>
#include <QApplication>

#include "rclcpp/rclcpp.hpp"
#include "parsian_gui/interface/interface_node.h"
#include "parsian_gui/interface/application/main_window.h"


struct NodeInThread
{
    NodeInThread(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        interface_node = std::make_shared<InterfaceNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    }
    void operator()(){
        rclcpp::spin(interface_node);
        rclcpp::shutdown();
    }

    std::shared_ptr<InterfaceNode> interface_node;
};

int main(int argc, char * argv[])
{
    // run interface-node in another thread
    NodeInThread node_in_thread{argc, argv};
    std::thread node_thread{node_in_thread};
    node_thread.detach();

    //create interface application
    QApplication a{argc, argv};
    MainWindow w{argc, argv, node_in_thread.interface_node};
    w.show();
    a.exec();




    return 0;
}
