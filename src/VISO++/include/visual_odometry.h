#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "common_include.h"
#include "map.h"
#include <opencv2/features2d/features2d.hpp>

class VisualOdometry
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
    Frame::Ptr              ref_;                  /*reference frame*/
    Frame::Ptr              curr_;                 /*current frame*/

    cv::Ptr<cv::ORB>        orb_;                  /*orb detector and computer*/
    vector<cv::Point3f>     pts_3d_ref_;           /*3d points in reference frame*/
    vector<cv::KeyPoint>    keypoints_ref_;        /*keypoints in ref frame*/
    vector<cv::KeyPoint>    keypoints_curr_;       /*keypoints in current frame*/


    Mat                     descriptors_curr_;     /*descriptor in current frame*/
    Mat                     descriptors_ref_;      /*descriptor in reference frame*/
    vector<cv::DMatch>      feature_matches_;      /**/
    cv::FlannBasedMatcher   matcher_flann_;        /* flann matcher */
    vector<MapPoint::Ptr>   match_3dpts_;          /*matched 3d points*/
    vector<int>             match_2dkp_index_;     /*matched 2d pixels (index of kp_curr)*/

    SE3                     T_c_w_estimated_;      /*the estimated pose of current frame cuz using map points to calc the T */
    int                     num_inliers_;          /*number of inlier features */
    int                     num_lost_;             /*number of lost times*/

    int                     num_of_features_;      /*number of features*/
    double                  scale_factor_;         /*scale in image pyramid*/
    int                     level_pyramid_;        /*number of pyramid levels*/
    float                   match_ratio_;          /*ratio for selecting good matches*/
    int                     max_num_lost_;         /*max number of continuos lost times*/
    int                     min_inliers_;          /*minimum inliers*/

    double                  key_frame_min_rot;     /*min rotation of two keyframes*/
    double                  key_frame_min_trans;   /*min translation of two keyframes*/
    double                  map_point_erase_ratio_;/*remove map point ratio*/

public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame( Frame::Ptr frame );

    /*functions are below*/
protected:

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


};



#endif // VISUAL_ODOMETRY_H
