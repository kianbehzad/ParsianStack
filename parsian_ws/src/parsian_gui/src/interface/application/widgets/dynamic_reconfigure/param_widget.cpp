//
// Created by kian behzad on 4/10/20.
//
#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/param_widget.h"



ParamWidget::ParamWidget(std::shared_ptr<rclcpp::SyncParametersClient> remote_param_client_, QWidget *parent) : QWidget(parent), remote_param_client{remote_param_client_}
{
    // getting style sheets
    QString qss_file_path = QDir(QString::fromStdString(extern_resources_directory_path)).filePath("param_widget.qss");
    File.setFileName(qss_file_path);
    if(!File.open(QFile::ReadOnly))
        RCLCPP_WARN(extern_interface_node->get_logger(), "[dynamic-reconfigure][param_widget] could not open param_widget.qss file!");
    FormStyleSheet = QLatin1String(File.readAll());
    this->setStyleSheet(FormStyleSheet);
    File.close();

    // define variables
    inner_layout = new QHBoxLayout();
    bounding_widget = new QWidget();
    outer_layout = new QHBoxLayout();

    bounding_widget->setObjectName("bounding_widget");

    label_param_name = new QLabel();
    label_param_name->setObjectName("label_param_name");
    label_param_name->setAlignment(Qt::AlignCenter);

    edit_param_value = new QLineEdit();
    edit_param_value->setObjectName("edit_param_value");
    edit_param_value->setAlignment(Qt::AlignCenter);
    edit_param_value->setProperty("is_edited", false);
    edit_param_value->style()->unpolish(edit_param_value);
    edit_param_value->style()->polish(edit_param_value);
    edit_param_value->update();

    button_submit_edit_param = new QPushButton();
    button_submit_edit_param->setObjectName("button_submit_edit_param");
    button_submit_edit_param->setText("submit");

    check_bool_param_value = new QCheckBox();
    check_bool_param_value->setObjectName("check_bool_param_value");

    int_validator_param_value = new QIntValidator();
    double_validator_param_value = new QDoubleValidator();


    // create layouts and widget
    inner_layout->addWidget(label_param_name);
    bounding_widget->setLayout(inner_layout);
    outer_layout->addWidget(bounding_widget);
    this->setLayout(outer_layout);


    //connections
    connect(this->check_bool_param_value, SIGNAL(stateChanged(int)), this, SLOT(check_bool_stateChanged_handle(int)));
    connect(this->button_submit_edit_param, SIGNAL(pressed()), this, SLOT(button_submit_pressed_handle()));
    connect(this->edit_param_value, SIGNAL(textEdited(QString)), this, SLOT(edit_param_textEdited_handle(QString)));



}

ParamWidget::~ParamWidget()
{
    delete label_param_name;
    delete edit_param_value;
    delete button_submit_edit_param;
    delete check_bool_param_value;
    delete int_validator_param_value;
    delete double_validator_param_value;
    delete inner_layout;
    delete bounding_widget;
    delete outer_layout;

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

            inner_layout->addWidget(edit_param_value);
            inner_layout->addWidget(button_submit_edit_param);
            break;
        }
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
        {
            double value = parameter.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE>();

            edit_param_value->setValidator(double_validator_param_value);
            edit_param_value->setText(QString::number(value));

            inner_layout->addWidget(edit_param_value);
            inner_layout->addWidget(button_submit_edit_param);
            break;
        }
        case rclcpp::ParameterType::PARAMETER_STRING:
        {
            QString value =  QString::fromStdString(parameter.get_value<rclcpp::ParameterType::PARAMETER_STRING>());

            edit_param_value->setText(value);

            inner_layout->addWidget(edit_param_value);
            inner_layout->addWidget(button_submit_edit_param);
            break;
        }
        case rclcpp::ParameterType::PARAMETER_BOOL:
        {
            bool value = parameter.get_value<rclcpp::ParameterType::PARAMETER_BOOL>();

            check_bool_param_value->setChecked(value);
            QString text = (value) ? "True" : "False";
            check_bool_param_value->setText(text);

            inner_layout->addItem(new QSpacerItem(0,5, QSizePolicy::Expanding, QSizePolicy::Expanding));
            inner_layout->addWidget(check_bool_param_value);
            break;
        }

        default: RCLCPP_WARN(extern_interface_node->get_logger(), "[dynamic-reconfigure] could not get the %s type", name.toStdString().c_str()); break;
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
    std::string current_text = edit_param_value->text().toStdString();
    auto set_parameters_results = remote_param_client->set_parameters({rclcpp::Parameter(get_name().toStdString(), current_text),});
    edit_param_value->setProperty("is_edited", false);
    edit_param_value->style()->unpolish(edit_param_value);
    edit_param_value->style()->polish(edit_param_value);
    edit_param_value->update();
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

void ParamWidget::edit_param_textEdited_handle(QString text)
{
    edit_param_value->setProperty("is_edited", true);
    edit_param_value->style()->unpolish(edit_param_value);
    edit_param_value->style()->polish(edit_param_value);
    edit_param_value->update();
}







