//
// Created by kian behzad on 4/9/20.
//

#ifndef PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
#define PARSIAN_GUI_DYNAMIC_RECONFIGURE_H

#include <vector>
#include <string>

#include <QWidget>
#include <QString>
#include <QDebug>
#include <QVBoxLayout>
#include <QPushButton>

#include "parsian_gui/interface/application/widgets/base_widget.h"

class DynamicReconfigureWidget : public BaseWidget
{
    Q_OBJECT
public:
    explicit DynamicReconfigureWidget(InterfaceNode* node_, std::vector<std::string> argv_, QWidget *parent = 0);
    ~DynamicReconfigureWidget();
    virtual void struct_widget();

};


#endif //PARSIAN_GUI_DYNAMIC_RECONFIGURE_H
