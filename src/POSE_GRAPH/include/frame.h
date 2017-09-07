#ifndef FRAME_H
#define FRAME_H
#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "converter.h"

class Matcher;
class VisualOdometry;
class VisualOdometryStereo;
class Frame
{

    /*actually this should be provite*/
public:

    //typedef std::shared_ptr<Frame>   Ptr;
    unsigned long                    mId;                            // id of the current frame
    Camera*                          mpCamera;                        // Pinhole RGBD camera model (for deteil please see camera.h)

    /*Image*/
    cv::Mat                          mImgLeft;
    cv::Mat                          mImgRight;

    Mat                              mTcl;                           // last -> curent


    // features and descriptor
    // u,v,val,class,d1~d8 descriptors

    std::vector<float>                mvDepth;
    std::vector<float>                mvuRight;


public:

    /*KeyPoints all points --> Current Left*/
    int                              N_total;                       // Number of KeyPoints sparse+dense.
    int                              N_circular;                    // Keypoints of circular matching
    int                              N_parallel;                    // Keypoints of left and right

    // MapPoints associated to keypoints, NULL pointer if no association.
    // 每个特征点对应的MapPoint maybe we need stereo matched mappoints's descriptor
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    std::vector<bool> mvbOutlier;

    //当前帧优化之后认为是inliers的点
    std::vector<int32_t> mvInliers;

public: //actually it should be privite
    SE3                              mT_c_w;                         // transform from world to camera
    Mat                              mTcw;                           // Opencv type, transform from world to camera
    cv::Mat                          mRcw;                           // Rotation from world to camera
    cv::Mat                          mtcw;                           // Translation from world to camera
    cv::Mat                          mRwc;                           // Rotation from camera to world
    cv::Mat                          mOw;                            // mtwc,Translation from camera to world


public:
    Frame();



    Frame(long id,
            Camera* camera,
            Mat imLeft,
            Mat imRight
            );

    // Copy constructor.
    Frame(const Frame &frame);
    ~Frame();

    // get feature matching [Blob and Corner] on the image.
    std::vector<Matcher::p_match> getCircularMatches();

    // get stereo matching.
    std::vector<Matcher::p_match> getStereoMatches();


    // compute the mvDepth
    void computeDepth();

    // for debug --> show the depth
    cv::Mat showDepth();

    // computeuRight
    void computeuRight();

    // compute the descriptors
    void computeDescriptor();

    // generate init mappoints
    void generateMappoints();

    // update the mappoints
    void updateCurrMappoints();

    // Set the camera pose.
    void setPose(cv::Mat Tcw); /*Set Tcw_*/

    // Set the camera rotation


    // Computes rotation, translation and camera center matrices from the camera pose.
    void updatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter()
    {
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat getRotationInverse()
    {
        return mRwc.clone();
    }

    bool isInFrustum(MapPoint *pMP);  /*usually use this one*/

    //judge a 3d point is in the frame
    bool isInFrame(const Eigen::Vector3d &pt_world);
    bool isInFrame(const cv::Mat &pt_world);


    /**
     * @brief 找到在 以x,y为中心,边长为2r的方形内的特征点
     * @param x --> 图像坐标u
     * @param y --> 图像坐标v
     * @param r --> 边长
     * @return满足条件的特征点的序号
     */
    vector<size_t> getFeatureInArea(const float &x, const float &y, const float &r );



    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    // 判断路标点是否在视野中
    // now we need modified the Mappoint class 2017.08.21 night
    // bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    //cv::Mat UnprojectStereo(const int &i);



};



#endif // FRAME_H
