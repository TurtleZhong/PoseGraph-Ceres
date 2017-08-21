#include "common_include.h"
#include "camera.h"
#include "config.h"

using namespace myslam;

int main(int argc, char *argv[])
{

    Config::setParameterFile("/home/m/ws_orb2/src/VISO2+/config/KITTI00-02.yaml");
    Camera::Ptr camera (new Camera);
    cout << "camera.bf_ = " << camera->bf_ << endl;

    /*SE3 test*/
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    cout << "R = " << endl << R << endl;
    Vector3d t(1,0,0);
    SO3 SO3_R(R);
    cout << "SO3_R = " << endl << SO3_R << endl;
    cout << "SO3_R = " << endl << SO3_R.matrix() << endl;

    SE3 SE3_Rt(R, t);
    cout << "SE3_Rt = " << endl << SE3_Rt << endl;

    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d


    return 0;
}
