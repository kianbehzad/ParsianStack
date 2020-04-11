//
// Created by kian behzad on 4/9/20.
//

#ifndef PARSIAN_GUI_BASE_WIDGET_H
#define PARSIAN_GUI_BASE_WIDGET_H

#include <vector>
#include <string>

#include <QWidget>
#include <QString>
#include <QDebug>

#include "parsian_gui/interface/interface_node.h"

#include "rclcpp/rclcpp.hpp"


class BaseWidget : public QWidget
{
    Q_OBJECT
public:
    explicit BaseWidget(InterfaceNode* node_, std::string qss_directory_path_, std::vector<std::string> argv_, QWidget *parent = 0);
    ~BaseWidget();
    virtual void struct_widget() = 0;


protected:
    InterfaceNode* node;
    std::string qss_directory_path;
    std::vector<std::string> argv;


};


#endif //PARSIAN_GUI_BASE_WIDGET_H
