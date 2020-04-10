//
// Created by kian behzad on 4/10/20.
//

#ifndef PARSIAN_GUI_PARAM_WIDGET_H
#define PARSIAN_GUI_PARAM_WIDGET_H

#include <vector>
#include <string>

#include <QWidget>
#include <QString>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QIntValidator>
#include <QDoubleValidator>

#include "parsian_gui/interface/interface_node.h"

#include "rclcpp/rclcpp.hpp"


class ParamWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ParamWidget(InterfaceNode* node_, QWidget *parent = 0);
    ~ParamWidget();

    void struct_widget(const rclcpp::Parameter parameter_);

    QString get_name();
    rclcpp::ParameterType get_type();
    rclcpp::Parameter get_parameter();

private:
    InterfaceNode* node;
    rclcpp::Parameter parameter;
    rclcpp::ParameterType type;
    QString name;

    //gui
    QHBoxLayout* lay;
    QLabel* label_param_name;
    QLineEdit* edit_param_value;
    QPushButton* button_submit_edit_param;
    QCheckBox* check_bool_param_value;
    QIntValidator* int_validator_param_value;
    QDoubleValidator* double_validator_param_value;

public slots:
    void button_submit_pressed_handle();
    void check_bool_stateChanged_handle(int state);

};


#endif //PARSIAN_GUI_PARAM_WIDGET_H
