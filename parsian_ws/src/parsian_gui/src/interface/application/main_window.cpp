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
//    QWidget *temp = ui->tabWidget->widget(0);
//    QString text = ui->tabWidget->tabText(0);
//    ui->tabWidget->widget(0)->setUpdatesEnabled(false);
//    ui->tabWidget->removeTab(0);
//    ui->tabWidget->insertTab(0, new QWidget, text);
//    delete temp;

}

MainWindow::~MainWindow()
{
}

void MainWindow::handle_current_changed(int index)
{
    disconnect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(handle_current_changed(int)));

    if(index > ui->tabWidget->count() || index < 0)
        return;

    QString text = ui->tabWidget->tabText(index);
    for(int i{}; i < ui->tabWidget->count(); i++)
    {
        QWidget *temp = ui->tabWidget->widget(i);
        QString tmptext = ui->tabWidget->tabText(i);
        ui->tabWidget->widget(i)->setUpdatesEnabled(false);
        ui->tabWidget->removeTab(i);
        if(index != i)
            ui->tabWidget->insertTab(i, new QWidget, tmptext);
        delete temp;
    }
    if(text == "plotter")
        ui->tabWidget->insertTab(index, new Plotter, "plotter");
    else if(text == "graphic")
        ui->tabWidget->insertTab(index, new QWidget, "graphic");
    else
        ui->tabWidget->insertTab(index, new QWidget, "nothing");
    ui->tabWidget->setCurrentIndex(index);

    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(handle_current_changed(int)));


}



