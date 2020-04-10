//
// Created by kian behzad on 4/9/20.
//

#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/dynamic_reconfigure.h"

DynamicReconfigureWidget::DynamicReconfigureWidget(InterfaceNode* node_, std::vector<std::string> argv_, QWidget *parent) : BaseWidget(node_, argv_, parent)
{
    // finding parameter file
    std::string file_path{};
    for(int i{}; i < argv.size(); i++)
        if(argv[i] == "--params-file") {
            if (i + 1 <= argv.size())
                file_path = argv[i + 1];
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


}

DynamicReconfigureWidget::~DynamicReconfigureWidget()
{

}


void DynamicReconfigureWidget::struct_widget()
{

}

void DynamicReconfigureWidget::yaml_parser(std::string file_path) {

    rcutils_allocator_t tt = rcutils_get_default_allocator();
    rcl_params_t *test = rcl_yaml_node_struct_init(tt);
    qDebug() << rcl_parse_yaml_file(file_path.c_str(), test);
    std::unordered_map<std::string, std::vector<rclcpp::Parameter>> pamap = rclcpp::parameter_map_from(test);
}






