

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "config.h"
#include "tracking.h"
//#include "g2o_types.h"


Tracking::Tracking() :
    state_ ( INITIALIZING ),
    ref_ (),
    curr_ (),
    last_(),
    map_ ( new Map ),
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

bool Tracking::addFrame ( int id, Camera::Ptr camera, Mat imLeft, Mat imRight )
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
        // very important!!!
        if(curr_.mId > 6)
            trackLocalMap();


        if ( checkEstimatedPose() == true ) // a good estimation
        {

            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
                addLocalMapPoints();
                cout << BOLDYELLOW"Frame :" << curr_.mId << " is a key Frame!!" << endl;
            }

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
    if(curr_.mId > ref_.mId + 2)
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
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                        p_world, n, curr_.mvDescriptors[matchd.i1c] , &curr_
                    );
            map_->insertMapPoint( map_point );
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
    // add the local mappoints into map
    for(size_t i = 0; i < curr_.N_parallel; i++)
    {
        const Matcher::p_match &matchd = curr_.mvStereoMatches[i];

        Vector3d p_world = curr_.mpCamera->pixel2world(
                    Vector2d(matchd.u1c,matchd.v1c),curr_.mT_c_w,curr_.mvDepth[matchd.i1c]
                );
        Vector3d n = p_world - Converter::toVector3d(curr_.GetCameraCenter());
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
                    p_world, n, curr_.mvDescriptors[matchd.i1c] , &curr_
                );
        mvpLocalMapPoints.push_back(map_point);
    }
}

void Tracking::optimizeMap()
{

}

double Tracking::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    //    Vector3d n = point->pos_ - frame->getCamCenter();
    //    n.normalize();
    //    return acos( n.transpose()*point->norm_ );
    double tmp = 1.0;
    return tmp;
}


void Tracking::generateFrame(int id, Camera::Ptr camera, Mat imLeft, Mat imRight)
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

    /*Generate Mappoints */
    curr_.mvpMapPoints= vector<MapPoint::Ptr>(curr_.N_total,static_cast<MapPoint::Ptr>(NULL));
    curr_.generateMappoints();

    curr_.mvbOutlier = vector<bool>(curr_.N_total,false);

}

void Tracking::trackLocalMap()
{
    /*actually we need to update the local map points and use g2o to update the pose*/

    optimizer::PoseOptimization(curr_);


}

