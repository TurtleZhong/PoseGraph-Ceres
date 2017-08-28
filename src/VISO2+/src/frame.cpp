#include "frame.h"

Frame::Frame() //: mId(-1), mpCamera(nullptr)
{

}

Frame::Frame(long id,  Camera::Ptr camera, cv::Mat imLeft, cv::Mat imRight)
    : mId(id), mpCamera(camera), mImgLeft(imLeft), mImgRight(imRight)
{
    /*here we need to update the png data for viso to use*/
    mpimg_left_data = Converter::toPng(mImgLeft);
    mpimg_right_data = Converter::toPng(mImgRight);
    this->dims[0] = this->mImgLeft.cols;
    this->dims[1] = this->mImgLeft.rows;
    this->dims[2] = this->mImgLeft.cols;
    mparams.calib.f = mpCamera->fx_;
    mparams.calib.cu = mpCamera->cx_;
    mparams.calib.cv = mpCamera->cy_;
    mparams.base = mpCamera->b_;
    /*init the TCw*/
    this->mTcw = Mat::eye(4,4,CV_64FC1);
    this->mTcl = Mat::eye(4,4,CV_64FC1);
    this->updatePoseMatrices();
    /*actually here we can init all the members*/

}

/*copy constructer*/
Frame::Frame(const Frame &frame)
    :mId(frame.mId), mpCamera(frame.mpCamera),mvCircularMatches(frame.mvCircularMatches),mImgLeft(frame.mImgLeft),
     mImgRight(frame.mImgRight),mparams(frame.mparams),mpimg_left_data(frame.mpimg_left_data),
     mpimg_right_data(frame.mpimg_right_data),mTcl(frame.mTcl),mvfeatureCurrentLeft(frame.mvfeatureCurrentLeft),
     mvfeatureCurrentRight(frame.mvfeatureCurrentRight),mvfeaturePreviousLeft(frame.mvfeaturePreviousLeft),
     mvfeaturePreviousRight(frame.mvfeaturePreviousRight),
     mvDescriptors(frame.mvDescriptors),
     mvStereoMatches(frame.mvStereoMatches),N_total(frame.N_total),
     mvDepth(frame.mvDepth),
     N_circular(frame.N_circular),
     N_parallel(frame.N_parallel),
     mvpMapPoints(frame.mvpMapPoints),
     mvbOutlier(frame.mvbOutlier)
{
    this->dims[0] = frame.dims[0];
    this->dims[1] = frame.dims[1];
    this->dims[2] = frame.dims[2];

    if(!frame.mTcw.empty())
    {
        Mat T = frame.mTcw.clone();
        this->setPose(T);
    }
}

Frame::~Frame()
{

}


std::vector<Matcher::p_match> Frame::getCircularMatches()
{
    return mvCircularMatches;
}

std::vector<Matcher::p_match> Frame::getStereoMatches()
{
    return mvStereoMatches;
}

void Frame::computeDepth() //generate mvDepth
{
    if(mvfeatureCurrentLeft.size() > 0 && mvStereoMatches.size() > 0)
    {
        mvDepth = vector<float>(N_total,-1.0f);
        for(vector<Matcher::p_match>::const_iterator it = mvStereoMatches.begin(); it!=mvStereoMatches.end(); it++)
        {
            Matcher::p_match matches = *it;
            mvDepth[matches.i1c] = mpCamera->bf_ / (matches.u1c - matches.u2c);
        }
    }
    else
    {
        cerr << "mvStereoMatches is empty or wrong, you shold check it!" << endl;
    }

}

void Frame::computeDescriptor()
{
    if(mvfeatureCurrentLeft.size()>0)
    {
        for(int i = 0; i < N_total; i++)
        {
            int32_t* desp = &(mvfeatureCurrentLeft[i].d1);
            vector<int32_t> d;
            d.reserve(8);
            for(int8_t i = 0; i < 7; i++)
            {
                d.push_back(*(desp + i));
            }
            mvDescriptors.push_back(d);
        }
    }
    else
    {
        cerr << "mvfeatureCurrentLeft is empty or wrong, you should check it!" << endl;
    }
}




void Frame::setPose(cv::Mat Tcw)
{
    /*here we need to update the SE3 but I have not test it yet*/
    this->mTcw = Tcw.clone();
    cout << "in set pose" << mTcw << endl;
    mT_c_w = Converter::toSE3(mTcw);
    updatePoseMatrices();
}

void Frame::updatePoseMatrices()
{
    // [x_camera 1] = [R|t]*[x_world 1]，坐标为齐次形式
    // x_camera = R*x_world + t
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t(); //旋转矩阵为正交矩阵，所以它的逆等于它的转置
    mtcw = mTcw.rowRange(0,3).col(3);
    // mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
    // mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系
    mOw = -mRcw.t()*mtcw;
}


bool Frame::isInFrame(const Eigen::Vector3d &pt_world)
{
    Vector3d p_cam = mpCamera->world2camera(pt_world, mT_c_w);
    if ( p_cam(2,0) < 0 )
    {
        return false;
    }
    Vector2d pixel = mpCamera->world2pixel(pt_world, mT_c_w);
    return pixel(0,0)>0 && pixel(1,0)>0
        && pixel(0,0)<mImgLeft.cols
        && pixel(1,0)<mImgLeft.rows;
}

bool Frame::isInFrame(const cv::Mat &pt_world)
{
    Vector3d pt_w = Converter::toVector3d(pt_world);
    return isInFrame(pt_w);
}


vector<size_t> Frame::getFeatureInArea(const float &x, const float &y, const float &r)
{
    vector<size_t> vIndices;
    vIndices.reserve(N_total);
    const int uMin = max((int)(x-r),0);
    const int uMax = min(mImgLeft.cols, (int)(x+r));
    const int vMin = max((int)(y-r),0);
    const int vMax = min(mImgLeft.rows, (int)(y+r));

    for(int index = 0; index < N_total; index++)
    {
        Matcher::maximum kp = mvfeatureCurrentLeft[index];
        /*now kp is a feature*/
        if(kp.u > uMin && kp.u <uMax)
        {
            if(kp.v > vMin && kp.v < vMax)
            {
                vIndices.push_back(index);
            }
        }

    }
    return vIndices;
}














