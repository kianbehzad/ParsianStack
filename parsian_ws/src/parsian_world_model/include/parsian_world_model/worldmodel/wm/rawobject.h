//
// Created by parsian-ai on 10/5/17.
//

#ifndef PARSIAN_WORLD_MODEL_RAWOBJECT_H
#define PARSIAN_WORLD_MODEL_RAWOBJECT_H

#include <parsian_util/geom/geom.h>
#include <parsian_world_model/worldmodel/wm/movingobject.h>

class MovingObject;

class CRawObject {
public:
    CRawObject(int frameCnt, Vector2D _pos, double orientation, int _ID, double _confidence, MovingObject* _ref = nullptr, int _cam_id = 0, double t = 0.0);
    CRawObject();
    Vector2D pos, dir;
    int frameCount;
    int ID;
    int cam_id;
    MovingObject* ref;
    bool updated;
    double confidence;
    double time;
    bool merged;
    int mergeCount;
};

#endif //PARSIAN_WORLD_MODEL_RAWOBJECT_H
