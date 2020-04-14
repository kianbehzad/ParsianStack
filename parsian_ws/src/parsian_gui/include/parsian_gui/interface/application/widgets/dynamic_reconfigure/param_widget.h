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
#include <QFile>
#include <QDir>
#include <QStyle>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QIntValidator>
#include <QDoubleValidator>
#include <QTimer>

#include "parsian_gui/interface/application/extern_variables.h"
#include "parsian_gui/interface/interface_node.h"

#include "rclcpp/rclcpp.hpp"


class ParamWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ParamWidget(std::shared_ptr<rclcpp::SyncParametersClient> remote_param_client_, QWidget *parent = 0);
    ~ParamWidget();

    void struct_widget(const rclcpp::Parameter parameter_);

    QString get_name();
    rclcpp::ParameterType get_type();
    rclcpp::Parameter get_parameter();

private:
    //STYLESHEET
    QFile File;
    QString FormStyleSheet;

    std::shared_ptr<rclcpp::SyncParametersClient> remote_param_client;
    rclcpp::Parameter parameter;
    rclcpp::ParameterType type;
    QString name;

    QTimer* timer_update_param;
    bool is_edited;

    //gui
    QHBoxLayout* inner_layout;
    QHBoxLayout* outer_layout;
    QWidget* bounding_widget;
    QLabel* label_param_name;
    QLineEdit* edit_param_value;
    QPushButton* button_bool_param_value;
    QPushButton* button_submit_edit_param;
    QIntValidator* int_validator_param_value;
    QDoubleValidator* double_validator_param_value;

public slots:
    void button_submit_pressed_handle();
    void button_bool_preesed_handle();
    void edit_param_textEdited_handle(QString text);
    void timer_update_timeout_handle();

};


#endif //PARSIAN_GUI_PARAM_WIDGET_H
