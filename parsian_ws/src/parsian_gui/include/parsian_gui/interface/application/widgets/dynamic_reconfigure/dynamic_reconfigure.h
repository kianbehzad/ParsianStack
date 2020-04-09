//
// Created by kian behzad on 4/9/20.
//

#ifndef PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
#define PARSIAN_GUI_DYNAMIC_RECONFIGURE_H

#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <yaml.h>

#include <QWidget>
#include <QString>
#include <QDebug>
#include <QVBoxLayout>
#include <QPushButton>
#include <QMap>
#include <QList>
#include <QFile>
#include <QDir>

#include "parsian_gui/interface/application/widgets/base_widget.h"


struct ParsedYaml{
    QMap<QString, QString> params;
    QString node_name;
};

class DynamicReconfigureWidget : public BaseWidget
{
    Q_OBJECT
public:
    explicit DynamicReconfigureWidget(InterfaceNode* node_, std::vector<std::string> argv_, QWidget *parent = 0);
    ~DynamicReconfigureWidget();
    virtual void struct_widget();

private:
    void yaml_parser(std::string file_path);
    ParsedYaml parse_one_node(yaml_parser_t& parser, yaml_token_t& token);
    QList<ParsedYaml> parsed_yaml;


};


#endif //PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
