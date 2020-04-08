//
// Created by kian behzad on 4/8/20.
//

#ifndef PARSIAN_GUI_MAIN_WINDOW_H
#define PARSIAN_GUI_MAIN_WINDOW_H

#include <vector>
#include <string>

#include <QMainWindow>
#include <QDebug>

#include "parsian_gui/interface/interface_node.h"
#include "parsian_msgs/msg/parsian_world_model.hpp"

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

    std::vector<std::string> argv;
};

#endif //PARSIAN_GUI_MAIN_WINDOW_H
