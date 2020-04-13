//
// Created by kian behzad on 4/9/20.
//

#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/dynamic_reconfigure.h"

DynamicReconfigureWidget::DynamicReconfigureWidget(QWidget *parent) : BaseWidget(parent)
{
    // finding parameter file
    std::string file_path{};
    for(int i{}; i < extern_argv.size(); i++)
        if(extern_argv[i] == "--params-file") {
            if (i + 1 <= extern_argv.size())
                file_path = extern_argv[i + 1];
            else
                RCLCPP_WARN(extern_interface_node->get_logger(), "[dynamic-reconfigure] parameter yaml file not found!");
        }
    // make a temp copy of file
    std::string directory_path = QFileInfo(QString::fromStdString(file_path)).absoluteDir().absolutePath().toStdString();
    std::string copy_path = QDir(QString::fromStdString(directory_path)).filePath("my_param.yaml").toStdString();
    if(QFile::exists(QString::fromStdString(copy_path)))
        QFile::remove(QString::fromStdString(copy_path));
    QFile::copy(QString::fromStdString(file_path), QString::fromStdString(copy_path));

    // parse yaml file
    yaml_parser(copy_path);

    //remove file
    QFile::remove(QString::fromStdString(copy_path));

    // make remote parameter clients
    client_node = rclcpp::Node::make_shared("set_parameters");
    define_parameter_clients();





}

DynamicReconfigureWidget::~DynamicReconfigureWidget()
{
    for(auto a : layout_for_params)
    {
        QLayoutItem *child;
        while ((child = a->takeAt(0)) != 0)
            delete child;
        delete a;
    }

    for(auto a : widget_for_params)
        delete a;

    for(auto a : scroll_for_params)
        delete a;

    delete tab_widget;

    delete layout_main;
}


void DynamicReconfigureWidget::struct_widget()
{

    tab_widget = new QTabWidget();
    for(const auto& node_ : parsed) {
        std::string node_name = node_.first;
        if (parameter_client[node_name]->service_is_ready())
        {

            scroll_for_params.append(new QScrollArea);
            widget_for_params.append(new QWidget);
            layout_for_params.append(new QVBoxLayout);

            for (const auto &param : node_.second) {
                ParamWidget *tmp = new ParamWidget(parameter_client[node_name]);
                tmp->struct_widget(param);
                layout_for_params.last()->addWidget(tmp);
            }

            widget_for_params.last()->setLayout(layout_for_params.last());
            scroll_for_params.last()->setWidget(widget_for_params.last());

            tab_widget->addTab(scroll_for_params.last(), QString::fromStdString(node_name));
        }
    }

    layout_main = new QHBoxLayout();
    layout_main->addWidget(tab_widget);

    this->setLayout(layout_main);

    // set margin height and ...
    layout_main->setSpacing(0);
    layout_main->setContentsMargins(0, 0, 0, 0);
    tab_widget->setContentsMargins(0, 0, 0, 0);
    for(const auto& scroll : scroll_for_params)
        scroll->setContentsMargins(0, 0, 0, 0);
    for(const auto& widg : widget_for_params)
        widg->setContentsMargins(0, 0, 0, 0);
    for(const auto& lay : layout_for_params) {
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(0);
    }
    this->setMaximumWidth(400);

}

void DynamicReconfigureWidget::yaml_parser(std::string file_path) {

    rcutils_allocator_t alocator = rcutils_get_default_allocator();
    rcl_params_t *param_st = rcl_yaml_node_struct_init(alocator);
    rcl_parse_yaml_file(file_path.c_str(), param_st);
    parsed = rclcpp::parameter_map_from(param_st);

}

void DynamicReconfigureWidget::define_parameter_clients()
{
    for(const auto& node_param : parsed)
    {
        std::string node_name = node_param.first;
        parameter_client[node_name] = std::make_shared<rclcpp::SyncParametersClient>(client_node, node_name);
        if (!parameter_client[node_name]->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(extern_interface_node->get_logger(), "[dynamic-reconfigure] couldnt get %s remote param!", node_name.c_str());
                rclcpp::shutdown();
            }
            RCLCPP_WARN(extern_interface_node->get_logger(), "[dynamic-reconfigure] %s remote param not available, waiting again...", node_name.c_str());
        }
    }

}








