#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "common_include.h"
#include "map.h"
#include "viso.h"
#include "viso_stereo.h"
#include "optimizer.h"
#include <opencv2/features2d/features2d.hpp>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"



class Map;
class Tracking
{
public:
    //typedef shared_ptr<VisualOdometry> Ptr;        /*shared ptr*/
    enum VOstate {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    /*data members are below*/
    VOstate                 state_;                /*current vo status*/
    Frame                   ref_;                  /*reference frame*/
    Frame                   curr_;                 /*current frame*/
    Frame                   last_;                 /*last frame*/
    Map*                    map_;                  /*map with all frames and mappoints*/

    std::vector<MapPoint*> mvpLocalMapPoints;

    int                     keyframeCount;


    int                     num_lost_;


    float                   mThDepth;              /*Threshold close/far points*/

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

    bool addFrame(int id, Camera* camera, cv::Mat imLeft, cv::Mat imRight);

    /*functions are below*/
public:

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

    void addLocalMapPoints();
    void optimizeMap();
    double getViewAngle( Frame* frame, MapPoint* point );

    void generateFrame(int id, Camera* camera, cv::Mat imLeft, cv::Mat imRight);

    void searchLocalPoints();
    void trackLocalMap();




};



#endif // VISUAL_ODOMETRY_H
