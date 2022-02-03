#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/MapPoint.h"
#include "myslam/Frame.h"

namespace myslam{

    class Map
    {
        public:
        typedef shared_ptr<Map> Ptr;
        unordered_map<unsigned long, MapPoint::Ptr> _map_points;
        unordered_map<unsigned long, Frame::Ptr> _keyframes;

        Map();

        void insertKeyFrame(Frame::Ptr frame);
        void insertMapPoint(MapPoint::Ptr map_point);

    };
}
    
#endif