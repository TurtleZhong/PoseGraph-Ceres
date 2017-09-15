#ifndef PROJECTION_H
#define PROJECTION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/rotation.h>
#include <ceres/ceres.h>
using namespace ceres;

/*here we need to relize the reproject function manually
 * quaternion  --> 3 dims but 4 params
 * translation --> 3 dims
 * points      --> 3 dims
 * predictions --> 2 dims
 */

/*suppose a point in the camera coordination Pl[xl,yl,zl] in last frame
 * we need to estimate the Tcl[Rcl | tcl]
 * first we need to transform the 3d point seen in last frame to current frame
 * Pc = Rcl * Pcl + tcl. Pc[xc,yc,zc]
 * then we need to transform the 3d point Pc to the image plane through camera intrinc params --> usually fx,fy,cx,cy
 * u = fx * x * invz + cx
 * v = fy * y * invz + cy
 * u,v is the predictions pixel
 */

template<typename T>
inline bool CamProjection(const T* Quaternion, const T* translation, const T* point, T* predictions)
{
//    /*here we use Eigen to convert the rotation matrix */

    Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(Quaternion);
    Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1>>(translation);
    Eigen::Matrix<T,3,1> p = Eigen::Map<const Eigen::Matrix<T,3,1>>(point);
    Eigen::Matrix<T,3,1> p_p = q*p + t;


    //Eigen::Vector3d p_p(25.2,45.3,10.2);



    /*project to the p_p to the image plane*/
    T x  = T(p_p(0));
    T y  = T(p_p(1));
    T z  = T(p_p(2));
    T fx = T(718.856);
    T fy = T(718.856);
    T cx = T(607.1928);
    T cy = T(185.2157);

    predictions[0] = (fx * x) / z + cx;
    predictions[1] = (fy * y) / z + cy;
    return true;
}

#endif // PROJECTION_H
