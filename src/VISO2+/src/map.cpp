#include "map.h"


void Map::insertKeyFrame(Frame frame)
{
    cout << "KeyFrame size = " << keyframes_.size() << endl;
    if(keyframes_.find(frame.mId) == keyframes_.end())
    {
        keyframes_.insert(make_pair(frame.mId, frame));
    }
    else
    {
        keyframes_[frame.mId] = frame;
    }
}

void Map::insertMapPoint(MapPoint* map_point)
{
    if( map_points_.find(map_point->mId) == map_points_.end())
    {
        map_points_.insert( make_pair(map_point->mId, map_point));

    }
    else
    {
        map_points_[map_point->mId] = map_point;
    }
}
