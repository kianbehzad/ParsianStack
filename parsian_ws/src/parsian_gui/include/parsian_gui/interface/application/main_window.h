//
// Created by kian behzad on 4/8/20.
//

#ifndef PARSIAN_GUI_MAIN_WINDOW_H
#define PARSIAN_GUI_MAIN_WINDOW_H

#include <vector>
#include <string>

#include <QMainWindow>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QDir>

#include "parsian_gui/interface/interface_node.h"
#include "parsian_gui/interface/application/extern_variables.h"
#include "parsian_gui/interface/application/widgets/dynamic_reconfigure/dynamic_reconfigure.h"
#include "parsian_gui/interface/application/widgets/plotter/plotter.h"
#include "parsian_gui/interface/application/widgets/graphical_client/soccerview.h"
#include "parsian_msgs/msg/parsian_world_model.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>


//forward declaration
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char * argv[], std::shared_ptr<InterfaceNode> interface_node_, QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    DynamicReconfigureWidget* dynamic_reconfigure;


//public slots:
//    void handle_current_changed(int index);


};

#endif //PARSIAN_GUI_MAIN_WINDOW_H
