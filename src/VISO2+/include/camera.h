#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"

namespace myslam
{
    class Camera
    {
    public:
        typedef std::shared_ptr<Camera>  Ptr;
        float                            fx_;
        float                            fy_;
        float                            cx_;
        float                            cy_;
        float                            bf_;           /* for stereo bf = b*f */
        float                            b_;
        float                            k1_;
        float                            k2_;
        float                            p1_;
        float                            p2_;
        int                              width_;
        int                              height_;
        int                              isStereo_;
        float                            depth_scale_;  /*rgbd depth image*/


        Camera ();
        Camera ( float fx, float fy, float cx, float cy, float bf, float b, float k1, float k2, float p1, float p2, float width, float height, int isStereo, float depth_scale=0 ) :
            fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), bf_(bf), b_(b), k1_(k1), k2_(k2), p1_(p1), p2_(p2), width_(width), height_(height), isStereo_(isStereo), depth_scale_ ( depth_scale )
        {}

        //coordinate transform

        Vector3d world2camera (const Vector3d& p_w, const SE3& T_c_w);
        Vector3d camera2world (const Vector3d& p_c, const SE3& T_c_w);
        Vector2d camera2pixel (const Vector3d& p_c);
        Vector3d pixel2camera (const Vector2d& p_p, double depth = 1);
        Vector3d pixel2world  (const Vector2d& p_p, const SE3& T_c_w, double depth = 1);
        Vector2d world2pixel  (const Vector3d& p_w, const SE3& T_c_w);
        Vector3d camera2camera(const Vector3d& p_l, const SE3& T_c_l); /*last frame --> current frame*/




    };


}

#endif // CAMERA_H
