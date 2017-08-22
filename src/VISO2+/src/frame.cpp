#include "frame.h"

Frame::Frame() : id_(-1), camera_(nullptr)
{

}

Frame::Frame(long id, Camera::Ptr camera, cv::Mat imLeft, cv::Mat imRight)
    : id_(id), camera_(camera), imLeft_(imLeft), imRight_(imRight)
{
    /*here we need to update the png data for viso to use*/
    img_left_data = Converter::toPng(imLeft_);
    img_right_data = Converter::toPng(imRight_);
    this->dims[0] = this->imLeft_.cols;
    this->dims[1] = this->imLeft_.rows;
    this->dims[2] = this->imLeft_.cols;

    /*May be we need to update something else but need to test*/
}

Frame::~Frame()
{

}

//Frame::Ptr Frame::createFrame(long id, Camera::Ptr camera, Mat imLeft, Mat imRight)
//{
//    static long factory_id = 0;
//    return Frame::Ptr (new Frame(factory_id++));
//}

std::vector<Matcher::p_match> Frame::getFeatureMatches()
{
    return matches_;
}

void Frame::setPose(cv::Mat Tcw)
{
    /*here we need to update the SE3 but I have not test it yet*/
    Tcw_ = Tcw.clone();
    T_c_w_ = Converter::toSE3(Tcw_);
    updatePoseMatrices();
}

void Frame::updatePoseMatrices()
{
    // [x_camera 1] = [R|t]*[x_world 1]，坐标为齐次形式
    // x_camera = R*x_world + t
    Rcw_ = Tcw_.rowRange(0,3).colRange(0,3);
    Rwc_ = Rcw_.t(); //旋转矩阵为正交矩阵，所以它的逆等于它的转置
    tcw_ = Tcw_.rowRange(0,3).col(3);
    // mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
    // mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系
    Ow_ = -Rcw_.t()*tcw_;
}


bool Frame::isInFrame(const Eigen::Vector3d &pt_world)
{
    Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
    if ( p_cam(2,0) < 0 )
    {
        return false;
    }
    Vector2d pixel = camera_->world2pixel(pt_world, T_c_w_);
    return pixel(0,0)>0 && pixel(1,0)>0
        && pixel(0,0)<imLeft_.cols
        && pixel(1,0)<imLeft_.rows;
}

bool Frame::isInFrame(const cv::Mat &pt_world)
{
    Vector3d pt_w = Converter::toVector3d(pt_world);
    return isInFrame(pt_w);
}
