#include "parsian_gui/interface/interface_node.h"

InterfaceNode::InterfaceNode(int argc, char * argv[], const rclcpp::NodeOptions & options) : Node("interface_node", options)
{
//    for(int i{}; i < argc; i++) {
//        if(strcmp(argv[i], "--params-file") == 0)
//            qDebug() << argv[i+1];
//    }

    //create application
    aplication_thread = new AplicationThread{this};
    std::thread my_thread{*aplication_thread};
    my_thread.detach();

}
