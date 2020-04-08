//
// Created by kian behzad on 4/8/20.
//
#include "parsian_gui/interface/application/main_window.h"

MainWindow::MainWindow(int _argc, char * _argv[], std::shared_ptr<InterfaceNode> interface_node_, QWidget *parent) : QMainWindow(parent), interface_node(interface_node_)
{
    // store argv
    for(int i{}; i < _argc; i++)
        argv.push_back(_argv[i]);

    // set up world_model callback (just a sample - no usage)
    worldmodel_subscription = interface_node->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&MainWindow::worldmodel_callback, this, _1));

}

MainWindow::~MainWindow()
{

}

void MainWindow::worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg)
{
}


