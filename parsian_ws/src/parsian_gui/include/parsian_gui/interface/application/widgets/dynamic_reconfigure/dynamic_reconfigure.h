//
// Created by kian behzad on 4/9/20.
//

#ifndef PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
#define PARSIAN_GUI_DYNAMIC_RECONFIGURE_H

#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>

#include <QWidget>
#include <QString>
#include <QDebug>
#include <QVBoxLayout>
#include <QMap>
#include <QList>
#include <QFile>
#include <QDir>

#include "parsian_gui/interface/application/widgets/base_widget.h"
#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/param_widget.h"

#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/parameter_map.hpp"



class DynamicReconfigureWidget : public BaseWidget
{
    Q_OBJECT
public:
    explicit DynamicReconfigureWidget(InterfaceNode* node_, std::vector<std::string> argv_, QWidget *parent = 0);
    ~DynamicReconfigureWidget();
    virtual void struct_widget();

private:
    void yaml_parser(std::string file_path);
    std::unordered_map<std::string, std::vector<rclcpp::Parameter>> parsed;



};


#endif //PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
