#ifndef FRAME_H
#define FRAME_H
#include "common_include.h"
#include "camera.h"


namespace myslam
{
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame>   Ptr;
    unsigned long                    id_;             // id of the current frame
    double                           time_stamp_;     // when it is recorded
    SE3                              T_c_w_;          // transform from world to camera
    Mat                              Tcw_;            // Opencv type
    Camera::Ptr                      camera_;         // Pinhole RGBD camera model (for deteil please see camera.h)
    Mat                              color_, depth_;  // color and depth image
    vector<cv::KeyPoint>             kp;

public:
    Frame();
    Frame(
          long id,
          double time_stamp = 0,
          SE3 T_c_w = SE3(),
          Camera::Ptr camera = nullptr,
          Mat color = Mat(),
          Mat depth = Mat()
            );
    ~Frame();

    static Frame::Ptr createFrame();

    /*find depth in depth image*/
    double findDepth( const cv::KeyPoint& kp  );

    /*get the camera center*/
    Vector3d getCamCenter() const;

    /*check if a point is in this frame*/
    bool isInFrame( const Vector3d& pt_world);

};



}


#endif // FRAME_H
