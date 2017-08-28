#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "frame.h"


int main(int argc, char *argv[])
{

    Config::setParameterFile("/home/m/ws_orb2/src/VISO2+/config/KITTI00-02.yaml");
    Camera::Ptr camera (new Camera);
    cout << "camera.bf_ = " << camera->bf_ << endl;

    /*****************************SE3 test******************************************/
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    cout << "R = " << endl << R << endl;
    Vector3d t(1,0,0);
    SO3 SO3_R(R);
    cout << "SO3_R = " << endl << SO3_R << endl;
    cout << "SO3_R = " << endl << SO3_R.matrix() << endl;

    SE3 SE3_Rt(R, t);
    cout << "SE3_Rt = " << endl << SE3_Rt << endl;

    //typedef Eigen::Matrix<double,6,1> Vector6d;


    /***************************Part2 frame.cpp test********************************/
    Mat Left = cv::imread("/home/m/KITTI/dataset/sequences/00/image_0/000000.png");
    Mat Right = cv::imread("/home/m/KITTI/dataset/sequences/00/image_1/000000.png");
//    Frame currentFrame(0, camera, Left, Right);
//    cv::imshow("currrent frame", currentFrame.mImgLeft);
//    cout << "Frame:\n";
//    cout << "Frame.camera.bf = " << currentFrame.mpCamera->bf_ << endl;

//    /*set pose*/
//    Mat T = Converter::toCvMat(SE3_Rt.matrix());
//    currentFrame.setPose(T);
//    cout << "Frame.Tcw =\n" <<  currentFrame.mT_c_w << endl;
//    cout << "Frame.Tcw =\n" <<  currentFrame.mTcw << endl;
//    for(int i = 0; i < Left.rows*Left.cols; i++)
//    {
//        cout << "Frame.imgdata " << currentFrame.mpimg_left_data[i] << endl;
//    }




    cv::waitKey(0);





    return 0;
}
