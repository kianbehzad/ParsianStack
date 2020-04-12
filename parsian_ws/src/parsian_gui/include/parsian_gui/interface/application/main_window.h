//
// Created by kian behzad on 4/8/20.
//

#ifndef PARSIAN_GUI_MAIN_WINDOW_H
#define PARSIAN_GUI_MAIN_WINDOW_H

#include <vector>
#include <string>

#include <QMainWindow>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QDir>

#include "parsian_gui/interface/interface_node.h"
#include "parsian_gui/interface/application/extern_variables.h"
#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/dynamic_reconfigure.h"
#include "parsian_msgs/msg/parsian_world_model.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char * argv[], std::shared_ptr<InterfaceNode> interface_node_, QWidget *parent = 0);
    ~MainWindow();

private:
    std::shared_ptr<InterfaceNode> interface_node;

    void worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_subscription;


    // widgets
    DynamicReconfigureWidget* dynamic_reconfigure_widget;

};

#endif //PARSIAN_GUI_MAIN_WINDOW_H
