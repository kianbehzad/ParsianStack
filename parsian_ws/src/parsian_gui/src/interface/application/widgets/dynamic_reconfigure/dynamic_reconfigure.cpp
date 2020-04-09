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

void DynamicReconfigureWidget::yaml_parser(std::string file_path)
{
    FILE *fh = fopen(file_path.c_str(), "r");
    yaml_parser_t parser;
    yaml_token_t  token;

    /* Initialize parser */
    if(!yaml_parser_initialize(&parser))
        RCLCPP_WARN(node->get_logger(), "[dynamic-reconfigure] failed to initialize parser!");
    if(fh == NULL)
        RCLCPP_WARN(node->get_logger(), "[dynamic-reconfigure] parameter yaml file failed to open!");

    /* Set input file */
    yaml_parser_set_input_file(&parser, fh);


    // start
    yaml_parser_scan(&parser, &token);
    yaml_parser_scan(&parser, &token);
    yaml_parser_scan(&parser, &token);
    while(true)
    {
        ParsedYaml tmp;
        tmp = parse_one_node(parser, token);
        parsed_yaml.append(tmp);
        yaml_parser_scan(&parser, &token);
        yaml_parser_scan(&parser, &token);
        if(token.type == YAML_BLOCK_END_TOKEN) {
            break;
        }
    }
    for(auto& p : parsed_yaml) {
        qDebug() << p.node_name;
        for (auto &n : p.params.keys())
            qDebug() << "   " << n << p.params[n];
    }



    /* Cleanup */
    yaml_parser_delete(&parser);
    fclose(fh);
}

ParsedYaml DynamicReconfigureWidget::parse_one_node(yaml_parser_t& parser, yaml_token_t& token)
{
    ParsedYaml tmp;
    yaml_parser_scan(&parser, &token);
    tmp.node_name = QString::fromStdString(std::string{(char*)token.data.scalar.value});
    yaml_parser_scan(&parser, &token);
    yaml_parser_scan(&parser, &token);
    yaml_parser_scan(&parser, &token);
    yaml_parser_scan(&parser, &token);
    // after ros__parameters:
    yaml_parser_scan(&parser, &token);
    yaml_parser_scan(&parser, &token);
    // start of parameters:
    while(true) {
        yaml_parser_scan(&parser, &token);
        if(token.type == YAML_BLOCK_END_TOKEN) {
            break;
        }
        yaml_parser_scan(&parser, &token);
        QString key = QString::fromStdString(std::string{(char *) token.data.scalar.value});
        yaml_parser_scan(&parser, &token);
        yaml_parser_scan(&parser, &token);
        QString value = QString::fromStdString(std::string{(char *) token.data.scalar.value});
        tmp.params[key] = value;
    }

    return tmp;

}





