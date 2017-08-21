#ifndef MAP_H
#define MAP_H
#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam
{

class Map
{
public:
    typedef shared_ptr<Map>     Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;
    unordered_map<unsigned long, Frame::Ptr>    keyframes_;

    Map() {}

    void insertKeyFrame( Frame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point);

};



}
#endif // MAP_H
