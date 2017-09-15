#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/rotation.h>



#include "projection.h"


//#include "common/tools/rotation.h"
//#include "common/projection.h"


class ReprojectionError
{
public:
    ReprojectionError(Eigen::Vector2d observation):observation_(observation){}

template<typename T>
    bool operator()(const T* const quaternion,
                    const T* const translation,
                    const T* const point,
                    T* residuals)const{
        // camera[0,1,2] are the angle-axis rotation
        T predictions[2];
        CamProjection(quaternion, translation, point, predictions);
        residuals[0] = predictions[0] - T(observation_(0));
        residuals[1] = predictions[1] - T(observation_(1));



        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d observation){
        return (new ceres::AutoDiffCostFunction<ReprojectionError,2,4,3,3>(
            new ReprojectionError(observation)));
    }


private:
    Eigen::Vector2d observation_;
};








#endif // SnavelyReprojection.h

