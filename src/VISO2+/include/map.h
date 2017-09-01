#ifndef MAP_H
#define MAP_H
#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

class MapPoint;
class Map
{
public:
    typedef shared_ptr<Map>     Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;
    unordered_map<unsigned long, Frame>    keyframes_;

    Map() {}

    void insertKeyFrame(Frame frame );
    void insertMapPoint( MapPoint::Ptr map_point);

};

#endif // MAP_H
