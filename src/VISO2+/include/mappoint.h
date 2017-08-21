#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"

namespace myslam
{

class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint>  Ptr;
    unsigned long                 id_;             /*ID*/
    static unsigned long          factory_id_;     /*Factory ID*/
    bool                          good_;           /*weather a good point*/
    Vector3d                      pos_;            /*Position in world*/
    Vector3d                      norm_;           /*Normal of viewing direction*/
    Mat                           descriptor_;     /*Descriptor for matching*/

    list<Frame*>                  observed_frames_;/*key-frames that can be observe this point*/

    int                           matched_times_;  /*being an inliner in pose estimation*/
    int                           visible_times_;  /*being visible in current frame*/


public:
    MapPoint();
    MapPoint(long id,
            const Eigen::Vector3d &position,
            const Eigen::Vector3d &norm,
            Frame* frame = nullptr,
            const Mat& descriptor_ = Mat()
            );

    inline cv::Point3f getPositionCV() const {
        return cv::Point3f(pos_(0,0), pos_(1,0), pos_(2,0));
    }
    static MapPoint::Ptr createMapPoint();

    static MapPoint::Ptr createMapPoint(
            const Vector3d& pos_world,
            const Vector3d& norm_,
            const Mat& descriptor_,
            Frame* frame
            );

};

}


#endif // MAPPOINT_H
