//
// Created by kian behzad on 4/8/20.
//
#include "parsian_gui/interface/application/main_window.h"


//define extern variables

std::vector<std::string> extern_argv;
std::string extern_qss_directory_path;

MainWindow::MainWindow(int _argc, char * _argv[], std::shared_ptr<InterfaceNode> interface_node_, QWidget *parent) : QMainWindow(parent), interface_node(interface_node_)
{
    // store argv
    for(int i{}; i < _argc; i++)
        extern_argv.push_back(_argv[i]);

    //get dir path
    extern_qss_directory_path = ament_index_cpp::get_package_share_directory("parsian_gui");
    extern_qss_directory_path = QDir(QString::fromStdString(extern_qss_directory_path)).filePath("qss").toStdString();

    // set up world_model callback (just a sample - no usage)
    worldmodel_subscription = interface_node->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&MainWindow::worldmodel_callback, this, _1));


    // struct widgets
    dynamic_reconfigure_widget = new DynamicReconfigureWidget(interface_node.get(), this);
    dynamic_reconfigure_widget->struct_widget();

    // add widgets
    this->setCentralWidget(dynamic_reconfigure_widget);

}

MainWindow::~MainWindow()
{
    delete  dynamic_reconfigure_widget;
}

void MainWindow::worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg)
{
}


