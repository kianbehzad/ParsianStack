//
// Created by kian behzad on 4/9/20.
//

#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/dynamic_reconfigure.h"

DynamicReconfigureWidget::DynamicReconfigureWidget(InterfaceNode* node_, QWidget *parent) : BaseWidget(node_, parent)
{
    // finding parameter file
    std::string file_path{};
    for(int i{}; i < extern_argv.size(); i++)
        if(extern_argv[i] == "--params-file") {
            if (i + 1 <= extern_argv.size())
                file_path = extern_argv[i + 1];
            else
                RCLCPP_WARN(node->get_logger(), "[dynamic-reconfigure] parameter yaml file not found!");
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

}


void DynamicReconfigureWidget::struct_widget()
{

    tab_widget = new QTabWidget();
    for(const auto& node_ : parsed)
    {
        std::string node_name  = node_.first;

        scroll_for_params.append(new QScrollArea);
        widget_for_params.append(new QWidget);
        layout_for_params.append(new QVBoxLayout);

        for(const auto& param : node_.second)
        {
            std::string param_name  = param.get_name();
            ParamWidget* tmp = new ParamWidget(node, parameter_client[param_name]);
            tmp->struct_widget(param);
            layout_for_params.last()->addWidget(tmp);
        }

        widget_for_params.last()->setLayout(layout_for_params.last());
        scroll_for_params.last()->setWidget(widget_for_params.last());

        tab_widget->addTab(scroll_for_params.last(), QString::fromStdString(node_name));
    }

    layout_main = new QHBoxLayout();
    layout_main->addWidget(tab_widget);

    this->setLayout(layout_main);

//    QVBoxLayout* m_layout = new QVBoxLayout();
//    QScrollArea* m_area = new QScrollArea;
//    QWidget* m_contents = new QWidget;
//    QVBoxLayout* m_contentsLayout = new QVBoxLayout{m_contents};
//    std::vector<QSpinBox*> m_spinBoxes;
//    for(int i{};i<10;i++)
//        m_spinBoxes.push_back(new QSpinBox);
//
//    m_layout->addWidget(m_area);
//    m_area->setWidget(m_contents);
//    for (auto & spinbox : m_spinBoxes)
//        m_contentsLayout->addWidget(spinbox);
//    m_contentsLayout->setSizeConstraint(QLayout::SetMinimumSize);
//
//    this->setLayout(m_layout);

//    QHBoxLayout* out_lay = new QHBoxLayout();
//    QScrollArea* scroll = new QScrollArea();
//    QWidget* widget = new QWidget();
//    QVBoxLayout* lay = new QVBoxLayout();
//
//    ParamWidget* param_widget1 = new ParamWidget(node, parameter_client["/grsim_node"]);
//    rclcpp::Parameter param1 = parsed["/grsim_node"][0];
//    param_widget1->struct_widget(param1);
//
//    ParamWidget* param_widget2 = new ParamWidget(node, parameter_client["/grsim_node"]);
//    rclcpp::Parameter param2 = parsed["/grsim_node"][1];
//    param_widget2->struct_widget(param2);
//
//    lay->addWidget(param_widget1);
//    lay->addWidget(param_widget2);
//
//    widget->setLayout(lay);
//
//    scroll->setWidget(widget);
//    out_lay->addWidget(scroll);
//    this->setLayout(out_lay);


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
        while (!parameter_client[node_name]->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "[dynamic-reconfigure] couldnt get %s remote param!", node_name.c_str());
                rclcpp::shutdown();
            }
            RCLCPP_WARN(node->get_logger(), "[dynamic-reconfigure] %s remote param not available, waiting again...", node_name.c_str());
        }
    }

}








