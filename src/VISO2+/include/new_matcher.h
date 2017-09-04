#ifndef NEW_MATCHER_H
#define NEW_MATCHER_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

class New_Matcher
{
public:
    New_Matcher();
    ~New_Matcher();

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &frame, const std::vector<MapPoint*> &vpMapPoints);

};

#endif // NEW_MATCHER_H
