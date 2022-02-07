#include "myslam/Map.h"

namespace myslam
{
    Map::Map()
    {
        
    }
    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        if(_keyframes.find(frame->_id) == _keyframes.end())
            _keyframes.insert(make_pair(frame->_id, frame));
        else
            _keyframes[frame->_id] = frame;
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point)
    {
        if(_map_points.find(map_point->_id) == _map_points.end())
            _map_points.insert(make_pair(map_point->_id, map_point));
        else
            _map_points[map_point->_id] = map_point;
    }
}