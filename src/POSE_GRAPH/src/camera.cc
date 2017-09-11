/*
 * This code is used to coordinate transform
 *
 * world coordinate <---> camera coordinate <---> pixel coordinate
 *       X                        x                      u
 *       Y                        y                      v
 *       Z                        z(1)
 *
*/

#include "camera.h"
#include "config.h"

namespace POSE_GRAPH
{
    Camera::Camera()
    {
        fx_           = Config::get<float>("Camera.fx");
        fy_           = Config::get<float>("Camera.fy");
        cx_           = Config::get<float>("Camera.cx");
        cy_           = Config::get<float>("Camera.cy");
        bf_           = Config::get<float>("Camera.bf");
        b_            = this->bf_ / this->fx_;
        k1_           = Config::get<float>("Camera.k1");
        k2_           = Config::get<float>("Camera.k2");
        p1_           = Config::get<float>("Camera.p1");
        p2_           = Config::get<float>("Camera.p2");
        width_        = Config::get<int>("Camera.width");
        height_       = Config::get<int>("Camera.height");
        isStereo_     = Config::get<int>("Camera.isStereo");
        depth_scale_  = Config::get<float>("Camera.depth_scale");

    }

    Vector3d Camera::world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3 &T_c_w)
    {
        return T_c_w * p_w;
    }

    Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const Sophus::SE3 &T_c_w)
    {
        return T_c_w.inverse() * p_c;
    }

    Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c)
    {
        /*
         * u = (fx * x)/ z + cx
         * v = (fy * y)/ z + cy
        */
        return Vector2d (
                    fx_ * p_c(0,0) / p_c(2,0) + cx_,
                    fy_ * p_c(1,0) / p_c(2,0) + cy_
                    );
    }

    Vector3d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth)
    {
        /*
         * x = (u - cx)*z / fx
         * v = (v - cy)*z / fy
        */
        return Vector3d (
                    ( p_p(0,0) - cx_ ) * depth / fx_,
                    ( p_p(1,0) - cy_ ) * depth / fy_,
                    depth
                    );
    }

    cv::Point3f Camera::pixel2camera(const cv::Point2f &p_p, float depth)
    {
        /*
         * x = (u - cx)*z / fx
         * v = (v - cy)*z / fy
        */
        return cv::Point3f (
                    ( p_p.x - cx_ ) * depth / fx_,
                    ( p_p.y - cy_ ) * depth / fy_,
                    depth
                    );
    }

    Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3 &T_c_w)
    {
        return camera2pixel( world2camera(p_w, T_c_w));
    }

    Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const Sophus::SE3 &T_c_w, double depth)
    {
        return camera2world( pixel2camera(p_p, depth), T_c_w);
    }

    Vector3d Camera::camera2camera(const Eigen::Vector3d &p_l, const Sophus::SE3 &T_c_l)
    {
        return T_c_l * p_l;
    }
}



