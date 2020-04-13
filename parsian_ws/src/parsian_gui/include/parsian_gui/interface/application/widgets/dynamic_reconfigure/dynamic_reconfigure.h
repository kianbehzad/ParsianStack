//
// Created by kian behzad on 4/9/20.
//

#ifndef PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
#define PARSIAN_GUI_DYNAMIC_RECONFIGURE_H

#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <map>

#include <QWidget>
#include <QString>
#include <QDebug>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QMap>
#include <QList>
#include <QFile>
#include <QDir>

#include "parsian_gui/interface/application/extern_variables.h"
#include "parsian_gui/interface/application/widgets/base_widget.h"
#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/param_widget.h"

#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/parameter_map.hpp"



class DynamicReconfigureWidget : public BaseWidget
{
    Q_OBJECT
public:
    explicit DynamicReconfigureWidget(QWidget *parent = 0);
    ~DynamicReconfigureWidget();
    virtual void struct_widget();

private:
    // map node nomes to their parameter client
    std::map<std::string, std::shared_ptr<rclcpp::SyncParametersClient>> parameter_client;
    void define_parameter_clients();

    void yaml_parser(std::string file_path);
    std::unordered_map<std::string, std::vector<rclcpp::Parameter>> parsed;

    // node for changing params
    std::shared_ptr<rclcpp::Node> client_node;

    // widgets
    QHBoxLayout* layout_main;
    QTabWidget* tab_widget;
    QList<QScrollArea*> scroll_for_params;
    QList<QWidget*> widget_for_params;
    QList<QVBoxLayout*> layout_for_params;




};


#endif //PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
