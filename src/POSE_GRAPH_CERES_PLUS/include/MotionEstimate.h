#ifndef MOTIONESTIMATE_H
#define MOTIONESTIMATE_H

#include "ceres/ceres.h"
#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "converter.h"

using namespace ceres;


namespace POSE_GRAPH
{

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


    /*project to the p_p to the image plane*/
    T x  = T(p_p(0));
    T y  = T(p_p(1));
    T z  = T(p_p(2));

    T fx = Config::get<T> ("Camera.fx");
    T fy = Config::get<T> ("Camera.fy");
    T cx = Config::get<T> ("Camera.cx");
    T cy = Config::get<T> ("Camera.cy");

    predictions[0] = (fx * x) / z + cx;
    predictions[1] = (fy * y) / z + cy;
    return true;
}




class ReprojectionError3Dto2D
{
public:
    ReprojectionError3Dto2D(Eigen::Vector2d observation):observed(observation){}

template<typename T>
    bool operator()(const T* const quaternion,
                    const T* const translation,
                    const T* const point,
                    T* residuals)const{
        T predictions[2];
        CamProjection(quaternion, translation, point, predictions);
        residuals[0] = predictions[0] - T(observed(0));
        residuals[1] = predictions[1] - T(observed(1));

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d observed){
        return (new ceres::AutoDiffCostFunction<ReprojectionError3Dto2D,2,4,3,3>(
            new ReprojectionError3Dto2D(observed)));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
    Eigen::Vector2d observed;
};



class MotionEstimate
{
public:
    /*get the rotation from the Essention Matrix
     * Notice that we use pnp to remove the outliers in test/ceres_pose_graph.cpp
     * so here we use inliers to re-estimate the rotation and use ceres to estimate the translation
     * may be we can get a better estimation
     */
    MotionEstimate();
    ~MotionEstimate();
    MotionEstimate(Frame &currentFrame, Frame &lastFrame, ceres::Problem* problems);
    void getRotationFromEssential();
    void BuildOptimizationProblem();
    void SolveOptimizationProblem();

    void Estimate();
    cv::Mat toCvMat();

    inline Eigen::Quaterniond getQuaternion() {return q_estimated;}
    inline Eigen::Vector3d getTranslation() {return t_estimated;}


private:
    Frame CurrentFrame;
    Frame LastFrame;
    ceres::Problem* problem;
    Eigen::Quaterniond q_estimated;
    Eigen::Vector3d t_estimated;
    vector<Eigen::Vector3d> vPoints3d;

};




}







#endif // MOTIONESTIMATE_H
