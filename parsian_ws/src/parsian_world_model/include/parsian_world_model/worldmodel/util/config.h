//
// Created by kian behzad on 4/6/20.
//

#ifndef PARSIAN_WORLD_MODEL_CONFIG_H
#define PARSIAN_WORLD_MODEL_CONFIG_H

class WorldModelConfig
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

extern WorldModelConfig m_config;
extern bool isSimulation;
extern bool isOurSideLeft;
extern bool isOurColorYellow;



#endif //PARSIAN_WORLD_MODEL_CONFIG_H
