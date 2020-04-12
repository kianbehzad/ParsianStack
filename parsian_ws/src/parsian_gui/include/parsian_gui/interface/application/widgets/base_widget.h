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

#include "parsian_gui/interface/application/extern_variables.h"
#include "parsian_gui/interface/interface_node.h"

#include "rclcpp/rclcpp.hpp"


class BaseWidget : public QWidget
{
    Q_OBJECT
public:
    explicit BaseWidget(InterfaceNode* node_, QWidget *parent = 0);
    ~BaseWidget();
    virtual void struct_widget() = 0;


protected:
    InterfaceNode* node;


};


#endif //PARSIAN_GUI_BASE_WIDGET_H
