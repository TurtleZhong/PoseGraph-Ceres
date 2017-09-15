#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "ceres/ceres.h"
#include <ceres/local_parameterization.h>

#include "ReprojectionError.h"
//#include "ceres_extensions.h"

/*This is a simple example of BA problems using Ceres*/
using namespace std;
using namespace ceres;

cv::Mat drawReprojection(cv::Mat &Tcl, std::vector<Eigen::Vector3d> &vP3d, std::vector<Eigen::Vector2d> &vP2d );


int main(int argc, char *argv[])
{
    /*first we need to load the 3D-2D data
     * suppose that we use point2f to describe the 2D data
     * and use cv::Mat(3,1,CV_64FC1) to describe the 3D points
     */
    ifstream inFile;
    inFile.open("/home/m/ws_orb2/src/ORB_SLAM2/3d-2dpairs/32.txt");

    cout << "Get 3d - 2d pairs." << endl;
    std::vector<Eigen::Vector2d> vP2d;
    std::vector<Eigen::Vector3d> vP3d;
    int num = 0;

    while(inFile.good())
    {
        double x,y,z,u,v;
        inFile >> x >> y >> z >> u >> v;

        Eigen::Vector3d p3d(x,y,z);
        Eigen::Vector2d p2d(u,v);

        vP3d.push_back(p3d);
        vP2d.push_back(p2d);

        inFile.get();
    }


    cout << "We got " << num << " 3d-2d pairs" << endl;

    double* points = vP3d[0].data();
    cout << points[0] << points[1] << points[2] << endl;

    Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond q(R1);
    Eigen::Vector3d t(0,0,0);

    Problem problem;

    ceres::LocalParameterization* quaternion_local_parameterization = new EigenQuaternionParameterization;


    for(int i = 0; i < vP2d.size(); i++)
    {
        CostFunction* cost_function;
        cost_function = ReprojectionError::Create(vP2d[i]);
        LossFunction* loss_function = new HuberLoss(1.0);

        problem.AddResidualBlock(cost_function,loss_function,q.coeffs().data(),t.data(),vP3d[i].data());

        problem.SetParameterization(q.coeffs().data(),quaternion_local_parameterization);

    }

    problem.SetParameterBlockConstant((double*)vP3d.data());
    //problem.SetParameterBlockConstant(t.data());

    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 200;

    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    std::cout << summary.FullReport() << "\n";

    /*next we need to get the result of optimiztion*/

    cout << "The rotition matrix is: \n " << q.toRotationMatrix() << endl;
    cout << "The translation matrix is: \n" << t << endl;

    Eigen::Matrix3d eigR = q.toRotationMatrix();
    cv::Mat trans = cv::Mat(3,1,CV_64F);
    trans.at<double>(0) = t(0);
    trans.at<double>(1) = t(1);
    trans.at<double>(2) = t(2);

    cv::Mat R = cv::Mat(3,3,CV_64F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            R.at<double>(i,j) = eigR(i,j);
        }
    }

    cv::Mat Tcl = cv::Mat::eye(4,4,CV_64F);
    R.copyTo(Tcl.rowRange(0,3).colRange(0,3));
    trans.copyTo(Tcl.rowRange(0,3).col(3));

    cout << "Tcl = \n" << Tcl << endl;

    cv::Mat output = drawReprojection(Tcl,vP3d,vP2d);

    cv::imshow("reprojection",output);

    cv::waitKey(0);



    return 0;
}


cv::Mat drawReprojection(cv::Mat &Tcl, std::vector<Eigen::Vector3d> &vP3d, std::vector<Eigen::Vector2d> &vP2d )
{
    cv::Mat currentFrame;

    currentFrame = cv::imread("/home/m/KITTI/dataset/sequences/00/image_0/000032.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::cvtColor(currentFrame,currentFrame,CV_GRAY2BGR);

    for(int i = 0; i < vP2d.size(); i++)
    {
        cv::Point2f pt;
        pt.x = vP2d[i](0);
        pt.y = vP2d[i](1);

        cv::circle(currentFrame,cv::Point2f(pt.x,pt.y),1,cv::Scalar(0,255,0),1,8);

        cout << "###" << "row:" << i+1 << " u = " << pt.x << " v = " << pt.y << endl;

        cv::Mat Rcl = Tcl.rowRange(0,3).colRange(0,3);
        cv::Mat tcl = Tcl.rowRange(0,3).col(3);
        cv::Mat Pl = cv::Mat::zeros(3,1,CV_64F);
        for(int row = 0; row < 3; row++)
        {
            Pl.at<double>(row) = vP3d[i](row);
        }


        cv::Mat Pc = Rcl * Pl + tcl;
        double fx = double(718.856);
        double fy = double(718.856);
        double cx = double(607.1928);
        double cy = double(185.2157);

        double u = (fx * Pc.at<double>(0)) / Pc.at<double>(2) + cx;
        double v = (fy * Pc.at<double>(1)) / Pc.at<double>(2) + cy;

        cout << "row:" << i+1 << " u = " << u << " v = " << v << endl;

        cv::Point2f pt_predict;
        pt_predict.x = (float)u;
        pt_predict.y = (float)v;

        cv::circle(currentFrame,pt,1,cv::Scalar(255,0,0),1,8);

        cv::line(currentFrame,pt,pt_predict,cv::Scalar(0,0,255),1,8);


    }

    return currentFrame;

}
