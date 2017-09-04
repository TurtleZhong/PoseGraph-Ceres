#ifndef FRAME_H
#define FRAME_H
#include "common_include.h"
#include "camera.h"
#include "viso_stereo.h" /*This file is contain the matcher */
#include "converter.h"

class Frame
{

    /*actually this should be provite*/
public:

    typedef std::shared_ptr<Frame>   Ptr;
    unsigned long                    id_;                            // id of the current frame
    //double                           time_stamp_;                    // when it is recorded

    Camera::Ptr                      camera_;                        // Pinhole RGBD camera model (for deteil please see camera.h)

    //Here we need the feature and descriptor. It is contained in "viso_stereo.h"
    std::vector<Matcher::p_match>    matches_;

    /*Image*/
    cv::Mat                          imLeft_;
    cv::Mat                          imRight_;


    /*params for liviso2*/
    int32_t                          dims[3];
    uint8_t*                         img_left_data;
    uint8_t*                         img_right_data;

    // Rotation, translation and camera center
    SE3                              T_c_w_;                         // transform from world to camera

    Mat                              Tcw_;                           // Opencv type, transform from world to camera
    cv::Mat                          Rcw_;                           // Rotation from world to camera
    cv::Mat                          tcw_;                           // Translation from world to camera
    cv::Mat                          Rwc_;                           // Rotation from camera to world
    cv::Mat                          Ow_;                            // mtwc,Translation from camera to world


public:

    int                              N_;                             //Keypoints of circular matching

    // MapPoints associated to keypoints, NULL pointer if no association.
    // 每个特征点对应的MapPoint
    //std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    std::vector<bool> mvbOutlier;


public:
    Frame();

    // Copy constructor.
    //Frame(const Frame &frame);
    Frame(
            long id,
            Camera::Ptr camera = nullptr,
            Mat imLeft = Mat(),
            Mat imRight = Mat()
            );
    ~Frame();

//    static Frame::Ptr createFrame(            long id,
//                                              Camera::Ptr camera = nullptr,
//                                              Mat imLeft = Mat(),
//                                              Mat imRight = Mat());

    /*get depth*/



    // get feature matching [Blob and Corner] on the image.
    std::vector<Matcher::p_match> getFeatureMatches();

    // Extract descriptors

    //void ExtractDescriptors(); /*maybe need some param*/

    // Set the camera pose.
    void setPose(cv::Mat Tcw); /*Set Tcw_*/

    // Computes rotation, translation and camera center matrices from the camera pose.
    void updatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter()
    {
        return Ow_.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat getRotationInverse()
    {
        return Rwc_.clone();
    }

    bool isInFrame(const Eigen::Vector3d &pt_world);
    bool isInFrame(const cv::Mat &pt_world);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    // 判断路标点是否在视野中
    // now we need modified the Mappoint class 2017.08.21 night
    // bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    //cv::Mat UnprojectStereo(const int &i);



};



#endif // FRAME_H
