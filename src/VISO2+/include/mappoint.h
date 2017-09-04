#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"
#include "converter.h"

class Converter;
class Frame;
class Map;
class MapPoint
{
public:
    //typedef shared_ptr<MapPoint>  Ptr;
    unsigned long                 mId;                   /*ID*/
    static unsigned long          mFactory_id;           /*Factory ID*/
    bool                          mbGood;                 /*weather a good point*/
    Vector3d                      mPos;                  /*Position in world*/
    Vector3d                      mNorm;                 /*Normal of viewing direction*/
    long unsigned int             mnLastFrameSeen;
    vector<int32_t>               mvDescriptor;           /*Descriptor for matching format:d1-->d8*/

    list<Frame*>                  mObserved_frames;      /*key-frames that can be observe this point*/

    int                           mMatched_times;        /*being an inliner in pose estimation*/
    int                           mVisible_times;        /*being visible in current frame*/
    bool                          mbTrackInView;
    float                         mTrackProjX;
    float                         mTrackProjXR;
    float                         mTrackProjY;


public:
    MapPoint();
    MapPoint(
             const Eigen::Vector3d &position,
             const Eigen::Vector3d &norm,
             long unsigned int     &LastFrameSeen,
             const vector<int32_t> &mDescriptor = vector<int32_t>(8)
             );


    inline std::vector<int32_t> getDescriptor()
    {
        return mvDescriptor;
    }

    inline cv::Mat GetWorldPos()
    {
        return Converter::toCvMat(mPos,1);
    }

};



#endif // MAPPOINT_H
