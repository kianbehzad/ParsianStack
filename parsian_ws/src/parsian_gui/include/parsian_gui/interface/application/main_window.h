//
// Created by kian behzad on 4/8/20.
//

#ifndef PARSIAN_GUI_MAIN_WINDOW_H
#define PARSIAN_GUI_MAIN_WINDOW_H

#include <QMainWindow>

#include "parsian_gui/interface/interface_node.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(InterfaceNode* interface_node_, QWidget *parent = 0);
    ~MainWindow();

private:
    InterfaceNode* interface_node;
};

#endif //PARSIAN_GUI_MAIN_WINDOW_H
