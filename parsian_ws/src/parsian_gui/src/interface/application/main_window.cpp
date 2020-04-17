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


    ui->setupUi(this);
    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(handle_current_changed(int)));

}

MainWindow::~MainWindow()
{
}

void MainWindow::handle_current_changed(int index)
{
    for(int i{}; i < ui->tabWidget->count(); i++)
        if(i != index)
        {
            ui->tabWidget->widget(i)->setUpdatesEnabled(false);
            ui->tabWidget->widget(i)->setEnabled(false);
        }
    ui->tabWidget->widget(index)->setUpdatesEnabled(false);
    ui->tabWidget->widget(index)->setEnabled(false);
}



