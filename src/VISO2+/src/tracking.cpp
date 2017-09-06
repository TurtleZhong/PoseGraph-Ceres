

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "config.h"
#include "tracking.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "converter.h"
#include "optimizer.h"
#include "new_matcher.h"
#include <cmath>

//#include "g2o_types.h"


Tracking::Tracking() :
    state_ ( INITIALIZING ),
    ref_ (),
    curr_ (),
    last_(),
    llast_(),
    map_ ( new Map ),
    keyframeCount(0),
    num_lost_(0)
{
    mThDepth                 = Config::get<float>("Camera.bf") * (float)Config::get<int>("ThDepth") / Config::get<float>("Camera.fx");
    num_of_features_         = Config::get<int> ( "number_of_features" );
    scale_factor_            = Config::get<double> ( "scale_factor" );
    level_pyramid_           = Config::get<int> ( "level_pyramid" );
    match_ratio_             = Config::get<float> ( "match_ratio" );
    max_num_lost_            = Config::get<float> ( "max_num_lost" );
    min_inliers_             = Config::get<int> ( "min_inliers" );
    key_frame_min_rot        = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans      = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_   = Config::get<double> ( "map_point_erase_ratio" );
    param_.calib.f            = Config::get<double>("Camera.fx");
    param_.calib.cu           = Config::get<double>("Camera.cx");
    param_.calib.cv           = Config::get<double>("Camera.cy");
    param_.base               = Config::get<double>("Camera.bf") / Config::get<float>("Camera.fx");

    /*now init the viso*/
    viso_                    = new VisualOdometryStereo(param_);
}

Tracking::~Tracking()
{

}

bool Tracking::addFrame (int id, Camera *camera, Mat imLeft, Mat imRight )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        state_ = OK;

        generateFrame(id,camera,imLeft,imRight); /*now the input frame is init*/

        // extract features from first frame and add them into map
        addKeyFrame();      // the first frame is a key-frame and need to add to the mappoint
        addLocalMapPoints();
        break;
    }
    case OK:
    {
        generateFrame(id,camera,imLeft,imRight); /*now the input frame is init*/

        //here we need to use g2o to optimize the camera pose
        if(curr_.mId > 4)
        {
            cout << "frame id = " << curr_.mId << " " << last_.mId << " " << llast_.mId << endl;
            trackLocalMap();
            cv::Mat Rcl = slerpRotation();

            Mat Rcw = Rcl * last_.mRcw;  //Rcw = Rcl * Rlw
            Mat tcw = curr_.mtcw.clone();

            Mat Tcw = cv::Mat::eye(4,4,CV_64FC1);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            //cout << BOLDYELLOW"T = \n" << T << endl;
            //curr_.setPose(Tcw);

            cout << BOLDMAGENTA"slerp_ Tcl = \n" << Tcw << endl;
            cout << "estimate Tcl = \n" << curr_.mTcw << endl;
        }


        cout << "trackLocal map is done!!!" << endl;

        if ( checkEstimatedPose() == true ) // a good estimation
        {

            /*now we need update the mappoints*/
            /*and we need to optimize the mappoints*/
            curr_.updateCurrMappoints();


            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
                addLocalMapPoints();
                //keyframeCount++;
                if(keyframeCount == 2 )
                {
                    //keyframeCount = 0;
                    //mvpLocalMapPoints.clear();
                }
                cout << BOLDYELLOW"Frame :" << curr_.mId << " is a key Frame!!" << endl;
            }

            llast_ = Frame(last_);
            last_ = Frame(curr_);

        }
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}

bool Tracking::checkEstimatedPose()
{
    return true;
}



bool Tracking::checkKeyFrame()
{
    if(curr_.mId > ref_.mId)
    {
        return true;
    }
    else
        return false;
}

void Tracking::addKeyFrame()
{
    if ( map_->keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for(size_t i = 0; i < curr_.N_parallel; i++)
        {
            const Matcher::p_match &matchd = curr_.mvStereoMatches[i];

            Vector3d p_world = curr_.mpCamera->pixel2world(
                        Vector2d(matchd.u1c,matchd.v1c),curr_.mT_c_w,curr_.mvDepth[matchd.i1c]
                    );
            Vector3d n = p_world - Converter::toVector3d(curr_.GetCameraCenter());
            n.normalize();
            MapPoint* pMP = new MapPoint(p_world, n, curr_.mId, curr_.mvDescriptors[matchd.i1c]);
            map_->insertMapPoint( pMP );
        }

        map_->insertKeyFrame ( curr_ );
        ref_ = Frame(curr_);
        last_ = Frame(curr_);
    }

    map_->insertKeyFrame ( curr_ );
    ref_ = Frame(curr_);
}

void Tracking::addLocalMapPoints()
{
    // it seems that we need to update the local map
    // add the local mappoints into map
    for(size_t i = 0; i < curr_.N_parallel; i++)
    {
        const Matcher::p_match &matchd = curr_.mvStereoMatches[i];

        Vector3d p_world = curr_.mpCamera->pixel2world(
                    Vector2d(matchd.u1c,matchd.v1c),curr_.mT_c_w,curr_.mvDepth[matchd.i1c]
                );
        Vector3d n = p_world - Converter::toVector3d(curr_.GetCameraCenter());
        n.normalize();
        MapPoint* pMP = new MapPoint(p_world, n, curr_.mId, curr_.mvDescriptors[matchd.i1c]);
        mvpLocalMapPoints.push_back(pMP);
    }
}

void Tracking::optimizeMap()
{

}

double Tracking::getViewAngle ( Frame* frame, MapPoint* point )
{
    //    Vector3d n = point->pos_ - frame->getCamCenter();
    //    n.normalize();
    //    return acos( n.transpose()*point->norm_ );
    double tmp = 1.0;
    return tmp;
}


void Tracking::generateFrame(int id, Camera* camera, Mat imLeft, Mat imRight)
{

    this->curr_ = Frame(id, camera, imLeft, imRight);
    /*now we got the original data*/
    /*then we need to extract something useful*/
    viso_->process(curr_.mpimg_left_data,curr_.mpimg_right_data,curr_.dims);

    //    cout << BOLDBLUE"pose = \n" << viso_->getMotion() << endl;

    /*get the features*/
    curr_.mvfeatureCurrentLeft = viso_->matcher->mvfeatureCurrentLeft;
    curr_.mvfeatureCurrentRight = viso_->matcher->mvfeatureCurrentRight;
    curr_.mvfeaturePreviousLeft = viso_->matcher->mvfeaturePreviousLeft;
    curr_.mvfeaturePreviousRight = viso_->matcher->mvfeaturePreviousRight;

    //curr_.N_total = curr_.mvfeatureCurrentLeft[0].size() + curr_.mvfeatureCurrentLeft[1].size();
    curr_.N_total = curr_.mvfeatureCurrentLeft.size();

    /*got the feature matches*/
    curr_.mvCircularMatches = viso_->getMatches();
    curr_.N_circular = curr_.mvCircularMatches.size();

    curr_.mvStereoMatches = viso_->p_stereoMatched;
    curr_.N_parallel = curr_.mvStereoMatches.size();

    curr_.computeDepth();

    /*got the matched right u value*/
    curr_.computeuRight();

    /*got the descriptor*/
    curr_.computeDescriptor();


    Matrix_ Tcl = viso_->getMotion();
    //cout << BOLDGREEN"Tcl = \n" << Tcl << endl;
    Mat T = Converter::toCvMat(Tcl);
    Mat Tcw = T * last_.mTcw;  //Tcw = Tcl * Tlw
    //cout << BOLDYELLOW"T = \n" << T << endl;
    curr_.setPose(Tcw);

    /*get inliers of the current frame*/
    curr_.mvInliers = viso_->getInlierIndices();

    /*Generate Mappoints */
    curr_.mvpMapPoints= vector<MapPoint*>(curr_.N_total,static_cast<MapPoint*>(NULL));
    curr_.generateMappoints();

    curr_.mvbOutlier = vector<bool>(curr_.N_total,false);


    Mat depthImg = curr_.showDepth();
    cv::imshow("depthImg",depthImg);
    cv::waitKey(27);

}


void Tracking::searchLocalPoints()
{
    // Do not search map points already matched
//    for(vector<MapPoint*>::iterator vit=curr_.mvpMapPoints.begin(), vend=curr_.mvpMapPoints.end(); vit!=vend; vit++)
//    {
//        MapPoint* pMP = *vit;
//        if(pMP)
//        {

//            pMP->IncreaseVisible();
//            pMP->mnLastFrameSeen = mCurrentFrame.mnId;
//            //pMP->mbTrackInView = false;

//        }
//    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == curr_.mId)
            continue;

        // Project (this fills MapPoint variables for matching)
        if(curr_.isInFrustum(pMP))
        {
            //pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        cout << BOLDRED"nToMatch = " << nToMatch << endl;
    }

    New_Matcher matcher;
    matcher.SearchByProjection(curr_, mvpLocalMapPoints);

}



void Tracking::trackLocalMap()
{
    /*actually we need to update the local map points and use g2o to update the pose*/

    /*first we need to update curr_.mvpMappoints through mvlocalMappoints*/
    searchLocalPoints();

    optimizer::PoseOptimization(&curr_, &last_);
    cout << "run here ... " << endl;


}

cv::Mat Tracking::slerpRotation()
{
    /*first we got q(t t-2)*/
    cv::Mat Rt_t_2 = curr_.mTcl.rowRange(0,3).colRange(0,3) * last_.mTcl.rowRange(0,3).colRange(0,3);
    Eigen::Quaterniond qt_t_2 = Converter::toQuaternion(Rt_t_2);
    Eigen::Quaterniond qt_t_1 = Converter::toQuaternion(curr_.mTcl.clone().rowRange(0,3).colRange(0,3));
    Eigen::Quaterniond qt_1_t_2 = Converter::toQuaternion(last_.mTcl.clone().rowRange(0,3).colRange(0,3));

    Eigen::Quaterniond nqt_t_1 = qt_1_t_2.inverse() * qt_t_2;



    double d = qt_t_1.dot(nqt_t_1);
    double absd;
    if(absd > 0)
        absd = d;
    else
        absd = -d;
    double t1,t2;
    if(absd > 1.0)
    {
        t1 = t2 = 0.5;
    }
    else
    {
        double theta = acos(absd);
        double sintheta = sin(theta);
        t1 = sin(0.5 * theta) / sintheta;
        t2 = sin(0.5 * theta) / sintheta;
    }
    if(d < 0)
    {
        t1 = -t1;
    }


    double w = t1 * qt_t_1.w() + t2 * nqt_t_1.w();
    double x = t1 * qt_t_1.x() + t2 * nqt_t_1.x();
    double y = t1 * qt_t_1.y() + t2 * nqt_t_1.y();
    double z = t1 * qt_t_1.z() + t2 * nqt_t_1.z();


    Eigen::Quaterniond Qt_t_1(w,x,y,z);
    Qt_t_1.normalized();
    Eigen::Matrix3d eigMat = Qt_t_1.toRotationMatrix();

    cv::Mat mRotation = Converter::toCvMat(eigMat);
    return mRotation;
}


void Tracking::updateCurrMappoints()
{
    /*cuz the pose is updated so we need to update the mappoint to get more accuracy camera's pose*/
    /*refer to the frame.cpp*/
}
