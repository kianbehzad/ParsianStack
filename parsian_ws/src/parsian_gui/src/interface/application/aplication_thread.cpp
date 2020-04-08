//
// Created by kian behzad on 4/8/20.
//

#include "parsian_gui/interface/application/aplication_thread.h"

//forward declaration
#include "parsian_gui/interface/interface_node.h"

AplicationThread::AplicationThread(int argc, char * argv[], InterfaceNode* node){
    this->interface_node = node;

    //create worldmodel subscriber(no usage)
    worldmodel_subscription = interface_node->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&AplicationThread::worldmodel_callback, this, _1));

}

void AplicationThread::operator()(){
    while(true) {
        qDebug() << "thr";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}

void AplicationThread::worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg)
{
}


