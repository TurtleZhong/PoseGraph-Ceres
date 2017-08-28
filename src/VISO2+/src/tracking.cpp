

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
    //ref_ (),
    //curr_ (),
    map_ ( new Map )
{
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

bool Tracking::addFrame ( Frame::Ptr frame )
{
//    switch ( state_ )
//    {
//    case INITIALIZING:
//    {
//        state_ = OK;
//        curr_ = ref_ = frame;
//        // extract features from first frame and add them into map
//        extractKeyPoints();
//        computeDescriptors();
//        addKeyFrame();      // the first frame is a key-frame
//        break;
//    }
//    case OK:
//    {
//        curr_ = frame;
//        curr_->T_c_w_ = ref_->T_c_w_;
//        extractKeyPoints();
//        computeDescriptors();
//        featureMatching();
//        poseEstimationPnP();
//        if ( checkEstimatedPose() == true ) // a good estimation
//        {
//            curr_->T_c_w_ = T_c_w_estimated_;
//            optimizeMap();
//            num_lost_ = 0;
//            if ( checkKeyFrame() == true ) // is a key-frame
//            {
//                addKeyFrame();
//            }
//        }
//        else // bad estimation due to various reasons
//        {
//            num_lost_++;
//            if ( num_lost_ > max_num_lost_ )
//            {
//                state_ = LOST;
//            }
//            return false;
//        }
//        break;
//    }
//    case LOST:
//    {
//        cout<<"vo has lost."<<endl;
//        break;
//    }
//    }

    return true;
}

void Tracking::extractKeyPoints()
{
//    boost::timer timer;
//    orb_->detect ( curr_->color_, keypoints_curr_ );
//    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}

void Tracking::computeDescriptors()
{
//    boost::timer timer;
//    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
//    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}

void Tracking::featureMatching()
{
//    boost::timer timer;
//    vector<cv::DMatch> matches;
//    // select the candidates in map
//    Mat desp_map;
//    vector<MapPoint::Ptr> candidate;
//    for ( auto& allpoints: map_->map_points_ )
//    {
//        MapPoint::Ptr& p = allpoints.second;
//        // check if p in curr frame image
//        if ( curr_->isInFrame(p->pos_) )
//        {
//            // add to candidate
//            p->visible_times_++;
//            candidate.push_back( p );
//            desp_map.push_back( p->descriptor_ );
//        }
//    }

//    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
//    // select the best matches
//    float min_dis = std::min_element (
//                        matches.begin(), matches.end(),
//                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
//    {
//        return m1.distance < m2.distance;
//    } )->distance;

//    match_3dpts_.clear();
//    match_2dkp_index_.clear();
//    for ( cv::DMatch& m : matches )
//    {
//        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
//        {
//            match_3dpts_.push_back( candidate[m.queryIdx] );
//            match_2dkp_index_.push_back( m.trainIdx );
//        }
//    }
//    cout<<"good matches: "<<match_3dpts_.size() <<endl;
//    cout<<"match cost time: "<<timer.elapsed() <<endl;
}

void Tracking::poseEstimationPnP()
{
//    // construct the 3d 2d observations
//    vector<cv::Point3f> pts3d;
//    vector<cv::Point2f> pts2d;

//    for ( int index:match_2dkp_index_ )
//    {
//        pts2d.push_back ( keypoints_curr_[index].pt );
//    }
//    for ( MapPoint::Ptr pt:match_3dpts_ )
//    {
//        pts3d.push_back( pt->getPositionCV() );
//    }

//    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
//              ref_->camera_->fx_, 0, ref_->camera_->cx_,
//              0, ref_->camera_->fy_, ref_->camera_->cy_,
//              0,0,1
//            );
//    Mat rvec, tvec, inliers;
//    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
//    num_inliers_ = inliers.rows;
//    cout<<"pnp inliers: "<<num_inliers_<<endl;
//    T_c_w_estimated_ = SE3 (
//                           SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
//                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
//                       );

//    // using bundle adjustment to optimize the pose
//    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
//    Block* solver_ptr = new Block ( linearSolver );
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm ( solver );

//    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
//    pose->setId ( 0 );
//    pose->setEstimate ( g2o::SE3Quat (
//        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
//    ));
//    optimizer.addVertex ( pose );

//    // edges
//    for ( int i=0; i<inliers.rows; i++ )
//    {
//        int index = inliers.at<int> ( i,0 );
//        // 3D -> 2D projection
//        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
//        edge->setId ( i );
//        edge->setVertex ( 0, pose );
//        edge->camera_ = curr_->camera_.get();
//        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
//        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
//        edge->setInformation ( Eigen::Matrix2d::Identity() );
//        optimizer.addEdge ( edge );
//        // set the inlier map points
//        match_3dpts_[index]->matched_times_++;
//    }

//    optimizer.initializeOptimization();
//    optimizer.optimize ( 10 );

//    T_c_w_estimated_ = SE3 (
//        pose->estimate().rotation(),
//        pose->estimate().translation()
//    );

//    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
}

bool Tracking::checkEstimatedPose()
{
//    // check if the estimated pose is good
//    if ( num_inliers_ < min_inliers_ )
//    {
//        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
//        return false;
//    }
//    // if the motion is too large, it is probably wrong
//    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
//    Sophus::Vector6d d = T_r_c.log();
//    if ( d.norm() > 5.0 )
//    {
//        cout<<"reject because motion is too large: "<<d.norm() <<endl;
//        return false;
//    }
    return true;
}

bool Tracking::checkKeyFrame()
{
//    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
//    Sophus::Vector6d d = T_r_c.log();
//    Vector3d trans = d.head<3>();
//    Vector3d rot = d.tail<3>();
//    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
//        return true;
    return false;
}

void Tracking::addKeyFrame()
{
//    if ( map_->keyframes_.empty() )
//    {
//        // first key-frame, add all 3d points into map
//        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
//        {
//            double d = curr_->findDepth ( keypoints_curr_[i] );
//            if ( d < 0 )
//                continue;
//            Vector3d p_world = ref_->camera_->pixel2world (
//                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
//            );
//            Vector3d n = p_world - ref_->getCamCenter();
//            n.normalize();
//            MapPoint::Ptr map_point = MapPoint::createMapPoint(
//                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
//            );
//            map_->insertMapPoint( map_point );
//        }
//    }

//    map_->insertKeyFrame ( curr_ );
//    ref_ = curr_;
}

void Tracking::addMapPoints()
{
//    // add the new map points into map
//    vector<bool> matched(keypoints_curr_.size(), false);
//    for ( int index:match_2dkp_index_ )
//        matched[index] = true;
//    for ( int i=0; i<keypoints_curr_.size(); i++ )
//    {
//        if ( matched[i] == true )
//            continue;
//        double d = ref_->findDepth ( keypoints_curr_[i] );
//        if ( d<0 )
//            continue;
//        Vector3d p_world = ref_->camera_->pixel2world (
//            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ),
//            curr_->T_c_w_, d
//        );
//        Vector3d n = p_world - ref_->getCamCenter();
//        n.normalize();
//        MapPoint::Ptr map_point = MapPoint::createMapPoint(
//            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
//        );
//        map_->insertMapPoint( map_point );
//    }
}

void Tracking::optimizeMap()
{
//    // remove the hardly seen and no visible points
//    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
//    {
//        if ( !curr_->isInFrame(iter->second->pos_) )
//        {
//            iter = map_->map_points_.erase(iter);
//            continue;
//        }
//        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
//        if ( match_ratio < map_point_erase_ratio_ )
//        {
//            iter = map_->map_points_.erase(iter);
//            continue;
//        }

//        double angle = getViewAngle( curr_, iter->second );
//        if ( angle > M_PI/6. )
//        {
//            iter = map_->map_points_.erase(iter);
//            continue;
//        }
//        if ( iter->second->good_ == false )
//        {
//            // TODO try triangulate this map point
//        }
//        iter++;
//    }

//    if ( match_2dkp_index_.size()<100 )
//        addMapPoints();
//    if ( map_->map_points_.size() > 1000 )
//    {
//        // TODO map is too large, remove some one
//        map_point_erase_ratio_ += 0.05;
//    }
//    else
//        map_point_erase_ratio_ = 0.1;
//    cout<<"map points: "<<map_->map_points_.size()<<endl;
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

    cout << BOLDBLUE"pose = \n" << viso_->getMotion() << endl;

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

    /*got the descriptor*/
    curr_.computeDescriptor();


    Matrix Tcl = viso_->getMotion();
    //cout << BOLDGREEN"Tcl = \n" << Tcl << endl;
    Mat T = Converter::toCvMat(Tcl);
    //Mat Tcw = T * ref_.mTcw;  //Tcw = Tcl * Tlw
    //cout << BOLDYELLOW"T = \n" << T << endl;
    curr_.setPose(T);


    /*Generate Mappoints */
    curr_.mvpMapPoints= vector<MapPoint*>(curr_.N_total,static_cast<MapPoint*>(NULL));


}

