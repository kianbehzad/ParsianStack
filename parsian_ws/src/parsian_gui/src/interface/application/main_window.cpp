//
// Created by kian behzad on 4/8/20.
//
#include "parsian_gui/interface/application/main_window.h"
//forward declaration
#include "ui_mainwindow.h"

//define extern variables
std::vector<std::string> extern_argv;
std::string extern_resources_directory_path;
std::shared_ptr<InterfaceNode> extern_interface_node;
bool extern_is_our_color_yellow;
bool extern_is_our_side_left;


MainWindow::MainWindow(int _argc, char * _argv[], std::shared_ptr<InterfaceNode> interface_node_, QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    qRegisterMetaType<QVector<int>>("QVector<int>");

    // store argv
    for(int i{}; i < _argc; i++)
        extern_argv.push_back(_argv[i]);

    //get dir path
    extern_resources_directory_path = ament_index_cpp::get_package_share_directory("parsian_gui");
    extern_resources_directory_path = QDir(QString::fromStdString(extern_resources_directory_path)).filePath("resources").toStdString();

    extern_interface_node = interface_node_;

    // set up worldmodel parameter client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(interface_node_.get(), "worldmodel_node");
    if (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(interface_node_.get()->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_WARN(interface_node_.get()->get_logger(), "/worldmodel_node remote param not available");
    }
    else {
        // initialize worldmodel parameters
        extern_is_our_color_yellow = parameters_client->get_parameter("is_our_color_yellow", extern_is_our_color_yellow);
        extern_is_our_side_left = parameters_client->get_parameter("is_our_side_left", extern_is_our_side_left);
    }

    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    // using soccerview and plotter is not efficient
//    ui->setupUi(this);
//    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(handle_current_changed(int)));

    // just run dynamic reconfigure
    dynamic_reconfigure = new DynamicReconfigureWidget;
    this->setCentralWidget(dynamic_reconfigure);



}

MainWindow::~MainWindow()
{
    delete dynamic_reconfigure;
}

//void MainWindow::handle_current_changed(int index)
//{
//    disconnect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(handle_current_changed(int)));
//
//    if(index > ui->tabWidget->count() || index < 0)
//        return;
//
//    QString text = ui->tabWidget->tabText(index);
//    for(int i{}; i < ui->tabWidget->count(); i++)
//    {
//        QWidget *temp = ui->tabWidget->widget(i);
//        QString tmptext = ui->tabWidget->tabText(i);
//        ui->tabWidget->widget(i)->setUpdatesEnabled(false);
//        ui->tabWidget->removeTab(i);
//        if(index != i)
//            ui->tabWidget->insertTab(i, new QWidget, tmptext);
//        delete temp;
//    }
//    if(text == "plotter")
//        ui->tabWidget->insertTab(index, new Plotter, "plotter");
//    else if(text == "graphic")
//        ui->tabWidget->insertTab(index, new GLSoccerView, "graphic");
//    else
//        ui->tabWidget->insertTab(index, new QWidget, "nothing");
//    ui->tabWidget->setCurrentIndex(index);
//
//    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(handle_current_changed(int)));
//
//
//}

void MainWindow::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "is_our_color_yellow")
            {
                extern_is_our_color_yellow = changed_parameter.value.bool_value;
                RCLCPP_INFO(extern_interface_node.get()->get_logger(), "changing color, is_our_color_yellow: %d", extern_is_our_color_yellow);
            }
            else if(changed_parameter.name == "is_our_side_left")
            {
                extern_is_our_side_left = changed_parameter.value.bool_value;
                RCLCPP_INFO(extern_interface_node.get()->get_logger(), "changing side, is_our_side_left: %d", extern_is_our_side_left);
            }

        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}



