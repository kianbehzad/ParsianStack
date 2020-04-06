//
// Created by kian behzad on 4/6/20.
//

#ifndef PARSIAN_WORLD_MODEL_CONFIG_H
#define PARSIAN_WORLD_MODEL_CONFIG_H

class CameraConfig
{
public:
    int camn_num;
    bool camera_0;
    bool camera_1;
    bool camera_2;
    bool camera_3;
    bool camera_4;
    bool camera_5;
    bool camera_6;
    bool camera_7;
};

extern CameraConfig extern_cameraConfig;
extern bool extern_isSimulation;
extern bool extern_isOurSideLeft;
extern bool extern_isOurColorYellow;



#endif //PARSIAN_WORLD_MODEL_CONFIG_H
