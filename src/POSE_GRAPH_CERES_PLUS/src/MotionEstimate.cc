#include "common_include.h"
#include "MotionEstimate.h"
#include "converter.h"
#include "config.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace ceres;

using namespace POSE_GRAPH;
using namespace cv;

MotionEstimate::MotionEstimate()
{
    /**/
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    q_estimated = Eigen::Quaterniond(R);
    t_estimated = Eigen::Vector3d(0,0,0);
}
MotionEstimate::~MotionEstimate()
{

}

MotionEstimate::MotionEstimate(Frame &currentFrame, Frame &lastFrame, Problem *problems)
    :CurrentFrame(currentFrame),LastFrame(lastFrame),problem(problems)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    q_estimated = Eigen::Quaterniond(R);
    t_estimated = Eigen::Vector3d(0,0,0);
}


void MotionEstimate::getRotationFromEssential()
{
    double cx = Config::get<double>("Camera.cx");
    double cy = Config::get<double>("Camera.cy");
    double fx = Config::get<double>("Camera.fx");

    Point2d principal_point(cx,cy);
    vector<Point2f> points_last;
    vector<Point2f> points_curr;

    map<int,int> matches = CurrentFrame.matchesId;

    for(map<int,int>::iterator match_iter= matches.begin(); match_iter!=matches.end(); match_iter++)
    {
        points_last.push_back(LastFrame.mvKeys[match_iter->second].pt);
        points_curr.push_back(CurrentFrame.mvKeys[match_iter->first].pt);
    }


    Mat essentialMatrix = findEssentialMat(points_last,points_curr,fx,principal_point,RANSAC,0.99,2);

    cv::Mat R,t;
    recoverPose(essentialMatrix,points_last,points_curr,R,t,fx,principal_point);

    cout << "estimated R = \n" << R << endl;
    cout <<R.type() << endl;

    Eigen::Matrix3d rotationMatrix = Converter::toMatrix3d(R);
    cout << "rotationMatrix = \n" << rotationMatrix << endl;

    q_estimated = Converter::toQuaternion(R);

    cout << "q_estimated = " << q_estimated.coeffs() << endl;


}

void MotionEstimate::BuildOptimizationProblem()
{
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization* quaternion_local_parameterization =
            new EigenQuaternionParameterization;
    std::vector<Eigen::Vector2d> observation;

    map<int,int> &matches = CurrentFrame.matchesId;
    for(map<int,int>::iterator match_iter = matches.begin(); match_iter!=matches.end();match_iter++ )
    {
        int curr_index = match_iter->first;
        int last_index = match_iter->second;

        Eigen::Vector3d points = Converter::toVector3d(LastFrame.mvpMapPoints[last_index]->GetWorldPos());
        vPoints3d.push_back(points);
        Eigen::Vector2d points2d(CurrentFrame.mvKeys[curr_index].pt.x, CurrentFrame.mvKeys[curr_index].pt.y);
        observation.push_back(points2d);

    }


    for(int i = 0; i < vPoints3d.size(); i++)
    {
        //Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        //const Eigen::Matrix<double, 6, 6> sqrt_information = information.llt().matrixL();
        // Ceres will take ownership of the pointer.
        ceres::CostFunction* cost_function =
                ReprojectionError3Dto2D::Create(observation[i]);

        problem->AddResidualBlock(cost_function, loss_function,
                                  q_estimated.coeffs().data(),
                                  t_estimated.data(),
                                  vPoints3d[i].data()
                                  );

        problem->SetParameterization(q_estimated.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    problem->SetParameterBlockConstant(q_estimated.coeffs().data());
    /*set 3d points constant*/
    for(int i = 0; i < vPoints3d.size(); i++)
    {
        problem->SetParameterBlockConstant(vPoints3d[i].data());
    }
}

void MotionEstimate::SolveOptimizationProblem()
{
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    //std::cout << BOLDGREEN << summary.FullReport() << '\n';

}

void MotionEstimate::Estimate()
{
    getRotationFromEssential();
    BuildOptimizationProblem();
    SolveOptimizationProblem();
}

cv::Mat MotionEstimate::toCvMat()
{
    cv::Mat Rcl = Converter::toCvMat(q_estimated);
    cv::Mat tcl = Converter::toCvMat(t_estimated);
    cv::Mat Tcl = cv::Mat::eye(4,4,CV_32FC1);
    Rcl.copyTo(Tcl.rowRange(0,3).colRange(0,3));
    tcl.copyTo(Tcl.rowRange(0,3).col(3));

    return Tcl;
}
