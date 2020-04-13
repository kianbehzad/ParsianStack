//
// Created by parsian-ai on 9/19/17.
//

#ifndef PARSIAN_WORLD_MODEL_WORLDMODEL_H
#define PARSIAN_WORLD_MODEL_WORLDMODEL_H

#include "QObject"
#include "QDebug"

#include "parsian_msgs/msg/parsian_world_model.hpp"
#include "parsian_msgs/msg/ssl_vision_detection.hpp"
#include "parsian_msgs/msg/ssl_vision_geometry.hpp"
#include "parsian_msgs/msg/parsian_robot.hpp"
#include "parsian_msgs/msg/ssl_vision_detection_robot.hpp"
#include "parsian_msgs/msg/ssl_vision_detection_ball.hpp"

#include <parsian_world_model/worldmodel/wm/visionclient.h>
#include <parsian_world_model/worldmodel/wm/halfworld.h>
#include <parsian_world_model/worldmodel/wm/ball.h>
#include <parsian_world_model/worldmodel/wm/robot.h>
#include "parsian_world_model/worldmodel/util/config.h"

#include "parsian_util/core/knowledge.h"

class WorldModel {
public:
    WorldModel();
    ~WorldModel();

    void updateDetection(const parsian_msgs::msg::SSLVisionDetection::SharedPtr&);
    void updateGeom(const parsian_msgs::msg::SSLVisionGeometry::SharedPtr&);
    void execute();
    void merge(int frame);
    void init();
    void setMode(bool isSimulation);

    double vForwardCmd[12], vNormalCmd[12], vAngCmd[12];

    parsian_msgs::msg::ParsianWorldModel::SharedPtr getParsianWorldModel();
    Robot* them[_MAX_NUM_PLAYERS];
    Robot* us[_MAX_NUM_PLAYERS];

private:
    parsian_msgs::msg::ParsianRobot rosRobots[_MAX_NUM_PLAYERS * 2];
    parsian_msgs::msg::ParsianRobot rosBall;
    CVisionClient *vc;

    CHalfWorld w;
    CHalfWorld mergedHalfWorld;

    CBall* ball;
    bool simulationMode;
    void run();
    void update(CHalfWorld*);
    void testFunc(const parsian_msgs::msg::SSLVisionDetection::SharedPtr & packet);
    void printRobotInfo(const parsian_msgs::msg::SSLVisionDetectionRobot &robot);

    void toParsianMessage(const Robot* _robot, int id);
    void toParsianMessage(const CBall* _ball);


    parsian_msgs::msg::SSLVisionDetection::SharedPtr detection;

    double visionFPS;
    double visionLatency;
    double visionTimestep;
    double visionProcessTime;

    int packs;


};


#endif //PARSIAN_WORLD_MODEL_WORLDMODEL_H
