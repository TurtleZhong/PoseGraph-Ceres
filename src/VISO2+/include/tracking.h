#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "common_include.h"
#include "map.h"
#include "viso.h"
#include "viso_stereo.h"
#include <opencv2/features2d/features2d.hpp>

class Tracking
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;        /*shared ptr*/
    enum VOstate {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    /*data members are below*/
    VOstate                 state_;                /*current vo status*/
    Map::Ptr                map_;                  /*map with all frames and mappoints*/
    Frame                   ref_;                  /*reference frame*/
    Frame                   curr_;                 /*current frame*/

    int                     num_of_features_;      /*number of features*/
    double                  scale_factor_;         /*scale in image pyramid*/
    int                     level_pyramid_;        /*number of pyramid levels*/
    float                   match_ratio_;          /*ratio for selecting good matches*/
    int                     max_num_lost_;         /*max number of continuos lost times*/
    int                     min_inliers_;          /*minimum inliers*/

    double                  key_frame_min_rot;     /*min rotation of two keyframes*/
    double                  key_frame_min_trans;   /*min translation of two keyframes*/
    double                  map_point_erase_ratio_;/*remove map point ratio*/

    /*new add --- zhong */
    VisualOdometryStereo::parameters param_;
    VisualOdometryStereo*   viso_;


public:
    Tracking();
    ~Tracking();

    bool addFrame( Frame::Ptr frame );

    /*functions are below*/
public:

    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

    void addMapPoints();
    void optimizeMap();
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );

    void generateFrame(int id, Camera::Ptr camera, cv::Mat imLeft, cv::Mat imRight);



};



#endif // VISUAL_ODOMETRY_H
