//
// Created by kian behzad on 4/9/20.
//

#include "parsian_gui/interface/application/widgets/base_widget.h"

BaseWidget::BaseWidget(InterfaceNode* node_, std::vector<std::string> argv_, QWidget *parent) : QWidget(parent), node{node_}, argv{argv_}
{
}

BaseWidget::~BaseWidget()
{

}

