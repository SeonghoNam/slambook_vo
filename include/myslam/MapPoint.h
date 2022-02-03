#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam{
    class MapPoint{
        public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long _id;
        Vector3d _pos;
        Vector3d _norm;
        Mat _descripter;
        int _observed_time;
        int _correct_time;

        MapPoint();
        MapPoint(long id, Vector3d pos, Vector3d norm);

        MapPoint::Ptr createMapPoint();
    };

}
#endif