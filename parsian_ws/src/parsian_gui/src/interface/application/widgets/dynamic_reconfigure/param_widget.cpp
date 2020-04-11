//
// Created by kian behzad on 4/10/20.
//
#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/param_widget.h"



ParamWidget::ParamWidget(InterfaceNode* node_, std::shared_ptr<rclcpp::SyncParametersClient> remote_param_client_, QWidget *parent) : QWidget(parent), node{node_}, remote_param_client{remote_param_client_}
{
    std::cout << remote_param_client_ << std::endl;
    std::cout << remote_param_client << std::endl;
    lay = new QHBoxLayout();
    label_param_name = new QLabel();
    edit_param_value = new QLineEdit();
    button_submit_edit_param = new QPushButton();
    check_bool_param_value = new QCheckBox();
    int_validator_param_value = new QIntValidator();
    double_validator_param_value = new QDoubleValidator();

    button_submit_edit_param->setText("submit");

    connect(this->check_bool_param_value, SIGNAL(stateChanged(int)), this, SLOT(check_bool_stateChanged_handle(int)));
    connect(this->button_submit_edit_param, SIGNAL(pressed()), this, SLOT(button_submit_pressed_handle()));

    lay->addWidget(label_param_name);

    this->setLayout(lay);
}

ParamWidget::~ParamWidget()
{
    delete lay;
    delete label_param_name;
    delete edit_param_value;
    delete check_bool_param_value;
    delete int_validator_param_value;
    delete double_validator_param_value;
}

void ParamWidget::struct_widget(const rclcpp::Parameter parameter_)
{
    parameter = parameter_;
    name = QString::fromStdString(parameter.get_name());
    type = parameter.get_type();

    label_param_name->setText(name);

    switch (type)
    {
        case rclcpp::ParameterType::PARAMETER_INTEGER:
        {
            int value = parameter.get_value<rclcpp::ParameterType::PARAMETER_INTEGER>();

            edit_param_value->setValidator(int_validator_param_value);
            edit_param_value->setText(QString::number(value));

            lay->addWidget(edit_param_value);
            lay->addWidget(button_submit_edit_param);
            break;
        }
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
        {
            double value = parameter.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE>();

            edit_param_value->setValidator(double_validator_param_value);
            edit_param_value->setText(QString::number(value));

            lay->addWidget(edit_param_value);
            lay->addWidget(button_submit_edit_param);
            break;
        }
        case rclcpp::ParameterType::PARAMETER_STRING:
        {
            QString value =  QString::fromStdString(parameter.get_value<rclcpp::ParameterType::PARAMETER_STRING>());

            edit_param_value->setText(value);

            lay->addWidget(edit_param_value);
            lay->addWidget(button_submit_edit_param);
            break;
        }
        case rclcpp::ParameterType::PARAMETER_BOOL:
        {
            bool value = parameter.get_value<rclcpp::ParameterType::PARAMETER_BOOL>();

            check_bool_param_value->setChecked(value);
            QString text = (value) ? "True" : "False";
            check_bool_param_value->setText(text);

            lay->addWidget(check_bool_param_value);
            break;
        }

        default: RCLCPP_WARN(this->node->get_logger(), "[dynamic-reconfigure] could not get the %s type", name.toStdString().c_str()); break;
    }
}

rclcpp::Parameter ParamWidget::get_parameter()
{
    return parameter;
}

QString ParamWidget::get_name()
{
    return name;
}

rclcpp::ParameterType ParamWidget::get_type()
{
    return type;
}

void ParamWidget::button_submit_pressed_handle()
{
    qDebug() << "here";
}

void ParamWidget::check_bool_stateChanged_handle(int state)
{
    if(state == Qt::Checked)
    {
        auto set_parameters_results = remote_param_client->set_parameters({rclcpp::Parameter(get_name().toStdString(), true),});
        check_bool_param_value->setText("True");
    }
    else if(state == Qt::Unchecked)
    {
        auto set_parameters_results = remote_param_client->set_parameters({rclcpp::Parameter(get_name().toStdString(), false),});
        check_bool_param_value->setText("False");
    }
}







