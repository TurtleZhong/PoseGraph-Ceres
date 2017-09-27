#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "camera.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"
#include "ORBmatcher.h"
#include "PoseGraph3dError.h"
#include "types.h"
#include "ImageRead.h"
#include "MotionEstimate.h"

/*ceres parts*/
#include <ceres/ceres.h>

#include <fstream>

using namespace std;
using namespace ceres;
using namespace POSE_GRAPH;


struct RESULT_OF_PNP
{
    cv::Mat rvec;
    cv::Mat tvec;
    int inliers;
};
double normofTransform( cv::Mat rvec, cv::Mat tvec );


void checkForPoseGraph(vector<Frame>& vFrames, Frame &currentFrame, VectorOfEdges &Edges);
void checkFrame(Frame &frame1, Frame &frame2, VectorOfEdges &Edges);
std::vector<Frame> getCandidateFrames(vector<Frame>& vFrames, Frame &currentFrame);
RESULT_OF_PNP motionEstimate(Frame &frame1, Frame &frame2);
void BuildOptimizationProblem(const VectorOfEdges& Edges,
                              MapOfPoses* poses, ceres::Problem* problem);
bool SolveOptimizationProblem(ceres::Problem* problem);
bool OutputPoses(const std::string& filename, const MapOfPoses& poses);
cv::Mat DrawFrameMatch(Frame &currentFrame, Frame &lastFrame);
vector<Mat> DrawReprojectionError(Frame &currentFrame, Frame &lastFrame, cv::Mat &Tcl);

ofstream outFileLoop;


int main(int argc, char *argv[])
{


    Config::setParameterFile("../config/config.yaml");
    string dir = Config::get<string>("sequence_dir");
    /*get the sequence length*/
    int sequenceLength = Config::get<int>("sequence_length");

    /*new add 2017.09.16 --> ceres*/
    Problem problem;
    MapOfPoses poses;
    VectorOfEdges Edges;


    SequenceRun* sequenceRun = new SequenceRun();
    vector<Frame> vFrames;

    ofstream outFile;
    outFile.open("../camera_poses.txt");
    outFileLoop.open("../result/Edges/edges_for_loop.txt");

    ImageReader im;

    for(int32_t i = 0; i < sequenceLength; i++)
    {

//        char base_name[256];
//        sprintf(base_name,"%06d.png",i);
//        string left_img_file_name = dir + "/image_0/" + base_name;
//        string right_img_file_name = dir + "/image_1/" + base_name;
//        Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
//        Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);

        Mat Left = cv::imread(im.getImagePath(i)[0],CV_LOAD_IMAGE_GRAYSCALE);
        Mat Right = cv::imread(im.getImagePath(i)[1],CV_LOAD_IMAGE_GRAYSCALE);
        cout << RESET"Process :" << i << endl;

        sequenceRun->GrabImageStereo(Left,Right,0.0);
        Frame currentFrame = sequenceRun->mCurrentFrame;

        /*add the frame vertex*/

        Pose3d currentPose =Converter::toPose3d(currentFrame.mTwc);
        poses[currentFrame.mnId] = currentPose;

        /*add vertex and edges to generate pose graph*/
        if(i > 0)
        {
            vector<Frame> vCandidateFrames = getCandidateFrames(vFrames,currentFrame);

            checkForPoseGraph(vCandidateFrames, currentFrame, Edges);
        }

        cv::imshow("currentFrame", currentFrame.mImageGray);


        cv::waitKey(27);

        vFrames.push_back(currentFrame);

        Mat pose = currentFrame.mTwc;

        //cout << "currentframe.mTwc = " << pose << endl;
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(i==2 && j==3)
                    outFile << pose.at<float>(i,j);
                else
                    outFile << pose.at<float>(i,j) << " ";

            }
        }
        outFile << endl;


    }

    OutputPoses("../result/trajectory/trajectory_origin.txt",poses);
    /*after the data were generated, we created the pose_graph */
    BuildOptimizationProblem(Edges,&poses,&problem);
    bool isSuscess = SolveOptimizationProblem(&problem);
    if(isSuscess)
        cout << "Optimizing Suscessfully!" << endl;
    else
        cout << "May be some problems!" << endl;

    // Output the poses to the file with format: id x y z q_x q_y q_z q_w.
    OutputPoses("../result/trajectory/trajectory_update_y_not_constant.txt",poses);

    /*suppose taht y is constant*/
    GroundTruth gd;
    int id = 0;

    for(MapOfPoses::iterator pose_iter = poses.begin(); pose_iter!=poses.end(); pose_iter++)
    {
        cv::Mat twc = gd.getFrametwc(id);

        pose_iter->second.p(1) = (double)twc.at<float>(1);

        id++;

    }

    OutputPoses("../result/trajectory/trajectory_update.txt",poses);


    return 0;
}

void checkForPoseGraph(vector<Frame> &vFrames, Frame &currentFrame, VectorOfEdges &Edges)
{
    for(size_t i = 0; i < vFrames.size(); i++)
    {
        checkFrame(vFrames[i], currentFrame, Edges);
    }


}

void checkFrame(Frame &frame1, Frame &frame2, VectorOfEdges &Edges)
{
    /*first we need to check this two frames --> use ORBmatcher*/

    int nmatches = 0;
    if(frame2.mnId > 1592 && frame2.mnId < 1631)//1631
    {
        ORBmatcher matcher(0.7,false);

        nmatches = matcher.MatcheTwoFrames(frame2,frame1,false); //5
        cout << RESET"Frame" << frame2.mnId << "--" <<"Frame" << frame1.mnId << " matches = " << nmatches << endl;
    }
    else if (frame2.mnId > 3306 && frame2.mnId < 4200) //3701
    {
        ORBmatcher matcher(0.7,false);

        nmatches = matcher.MatcheTwoFrames(frame2,frame1,false);
        //        cout << RESET"matches = " << nmatches << endl;
        cout << RESET"Frame" << frame2.mnId << "--" <<"Frame" << frame1.mnId << " matches = " << nmatches << endl;
    }
    else if (frame2.mnId > 4460 && frame2.mnId < 4526)
    {
        ORBmatcher matcher(0.7,false);

        nmatches = matcher.MatcheTwoFrames(frame2,frame1,false);
        cout << RESET"Frame" << frame2.mnId << "--" <<"Frame" << frame1.mnId << " matches = " << nmatches << endl;
    }

    if(frame2.mnId - frame1.mnId == 1)
    {

        /*use ceres to add edges*/

        Edge3d edge;
        edge.id_begin = frame2.mnId;
        edge.id_end = frame1.mnId;
        cv::Mat Tcl = frame2.mTcw*frame1.mTwc;
        Pose3d T = Converter::toPose3d(Tcl);
        edge.t_be = T;
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        edge.information = information;

        Edges.push_back(edge);
        cout << BOLDCYAN"Add a neaby edge!" << endl;


    }

    if(frame2.mnId - frame1.mnId > 1 && nmatches > 280)
    {

        /*now we check the motion and inliers of this two frame*/
        RESULT_OF_PNP result = motionEstimate(frame1,frame2);
        double norm = normofTransform(result.rvec,result.tvec);

        cout << BOLDMAGENTA"Frame:" << frame2.mnId << " pnp_result: " << "result.inliers = " << result.inliers << " norm = " << norm << endl;

        if(result.inliers > 100 && norm < 0.7)
        {
            /*here we need use the estimated pose but not the groundTruth*/
            if(!frame2.mHaveLoopEdge && !frame1.mHaveLoopEdge)
            {

                /*use ceres to optimize the pose between two frames*/
                Frame CurrentFrame(frame2);
                Frame LastFrame(frame1);
                ceres::Problem problem;
                MotionEstimate motionestimate2(CurrentFrame,LastFrame, &problem);
                motionestimate2.Estimate();

                cv::Mat Tcl = motionestimate2.toCvMat();





//                cv::Mat R;
//                cv::Rodrigues(result.rvec,R);
//                R.convertTo(R,CV_32FC1);
//                cv::Mat Tcl = Mat::eye(4,4,CV_32FC1);
//                R.copyTo(Tcl.rowRange(0,3).colRange(0,3));
//                result.tvec.copyTo(Tcl.rowRange(0,3).col(3));


                Edge3d edge;
                edge.id_begin = frame2.mnId;
                edge.id_end = frame1.mnId;
                /* new add 09.23 */
                //cv::Mat Tcl_gd = frame2.mTcw * frame1.mTwc;


                //            cv::Mat Tcl = frame2.mTcw*frame1.mTwc;
                Pose3d T = Converter::toPose3d(Tcl);

                //T.p(1) = (double)Tcl_gd.at<float>(1,3);


                edge.t_be = T;
                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
                edge.information = information;

                Edges.push_back(edge);
                cv::Mat imMatch = DrawFrameMatch(frame2,frame1);
                imshow("Match", imMatch);


                cout << BOLDYELLOW"add a new edge! " << frame2.mnId << " --> " << frame1.mnId <<  endl;
                if(frame2.mnId - frame1.mnId > 100)
                    outFileLoop << frame2.mnId << " " << frame1.mnId << endl;

                frame2.mHaveLoopEdge = true;
                frame1.mHaveLoopEdge = true;
            }
        }

    }

    cout << RESET"Frame:" << frame2.mnId << " have loop egde: " << frame2.mHaveLoopEdge << endl;

}


std::vector<Frame> getCandidateFrames(vector<Frame>& vFrames, Frame &currentFrame)
{
    /*here we need to get the candidate frames according to the Twc*/
    std::vector<Frame> candidates;
    //cout << BOLDGREEN"candidate frames id = ";
    /*we need add the last frame*/
    candidates.push_back(vFrames.back());

    if(currentFrame.mnId > 1592 && currentFrame.mnId < 1631) /*The first part 1631*/
    {
        for(vector<Frame>::const_iterator vit = vFrames.begin(); vit!=(vFrames.end()-1);vit++)
        {
            if(currentFrame.mnId - Frame(*vit).mnId > 1 && currentFrame.mnId - Frame(*vit).mnId < 500)
                continue;

            cv::Mat twc = Frame(*vit).GetCameraCenter();

            bool isInRange = currentFrame.isInSearchRange(twc);
            if(isInRange)
            {
                if(currentFrame.mnId - Frame(*vit).mnId >= 100 )
                {
                    candidates.push_back(*vit);
                    cout << RESET"Candiate frame id:" << Frame(*vit).mnId << " ";
                    cout << BOLDRED"may be we got a loop!" << endl;
                }

            }

        }
    }
    else if (currentFrame.mnId > 3306 && currentFrame.mnId < 4200) //3701
    {
        for(vector<Frame>::const_iterator vit = vFrames.begin(); vit!=(vFrames.end()-1);vit++)
        {
            if(currentFrame.mnId - Frame(*vit).mnId > 1 && currentFrame.mnId - Frame(*vit).mnId < 500)
                continue;

            cv::Mat twc = Frame(*vit).GetCameraCenter();

            bool isInRange = currentFrame.isInSearchRange(twc);
            if(isInRange)
            {
                if(currentFrame.mnId - Frame(*vit).mnId >= 100 )
                {
                    candidates.push_back(*vit);
                    cout << RESET"Candiate frame id:" << Frame(*vit).mnId << " ";
                    cout << BOLDRED"may be we got a loop!" << endl;
                }

            }
        }
    }
    else if (currentFrame.mnId > 4460 && currentFrame.mnId < 4526)
    {
        for(vector<Frame>::const_iterator vit = vFrames.begin(); vit!=(vFrames.end()-1);vit++)
        {
            if(currentFrame.mnId - Frame(*vit).mnId > 1 && currentFrame.mnId - Frame(*vit).mnId < 500)
                continue;

            cv::Mat twc = Frame(*vit).GetCameraCenter();

            bool isInRange = currentFrame.isInSearchRange(twc);
            if(isInRange)
            {
                if(currentFrame.mnId - Frame(*vit).mnId >= 100 )
                {
                    candidates.push_back(*vit);
                    cout << RESET"Candiate frame id:" << Frame(*vit).mnId << " ";
                    cout << BOLDRED"may be we got a loop!" << endl;
                }

            }

        }
    }

    return candidates;
}


/*we need a simple function to estimate the inliers and pose*/
RESULT_OF_PNP motionEstimate(Frame &frame1, Frame &frame2)
{
    RESULT_OF_PNP result;
    map<int,int> matches = frame2.matchesId;
    vector<cv::Point3f> pts_obj;
    vector< cv::Point2f > pts_img;

    Camera* pCamera = new Camera();

    for(map<int,int>::const_iterator mit = matches.begin(); mit!=matches.end(); mit++)
    {

        cv::Point2f p = frame1.mvKeys[mit->second].pt;
        pts_img.push_back( cv::Point2f( frame2.mvKeys[mit->first].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point2f pt ( p.x, p.y );
        float depth = frame1.mvDepth[mit->second];

        cv::Point3f pd = pCamera->pixel2camera(pt,depth);
        pts_obj.push_back( pd );
    }


    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }




    double camera_matrix_data[3][3] = {
        {(double)pCamera->fx_, 0, (double)pCamera->cx_},
        {0, (double)pCamera->fy_, (double)pCamera->cy_},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 200, 2.0, 0.99, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    //    cout << "inliers = " <<  endl << inliers << endl;

    /*remove the outliers*/
    map<int,int> matches_out;
    for(int i = 0; i < inliers.rows; i++)
    {
        int row = inliers.at<int>(i);
        map<int,int>::const_iterator map_iter_start = matches.begin();

        if (row == 0)
        {
            matches_out.insert(make_pair(map_iter_start->first,map_iter_start->second));
        }
        else
        {
            while(row)
            {
                map_iter_start++;
                row--;
            }
            matches_out.insert(make_pair(map_iter_start->first,map_iter_start->second));
        }


    }

    //cout << "matches out.size = " << matches_out.size() << endl;
    frame2.matchesId.clear();
    frame2.matchesId = matches_out;

    return result;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

void BuildOptimizationProblem(const VectorOfEdges& Edges,
                              MapOfPoses* poses, ceres::Problem* problem)
{

    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization* quaternion_local_parameterization =
            new EigenQuaternionParameterization;

    for (VectorOfEdges::const_iterator Edges_iter =Edges.begin();Edges_iter != Edges.end(); ++Edges_iter)
    {
        const Edge3d& edge = *Edges_iter;

        MapOfPoses::iterator pose_begin_iter = poses->find(edge.id_begin);
        MapOfPoses::iterator pose_end_iter = poses->find(edge.id_end);



        const Eigen::Matrix<double, 6, 6> sqrt_information = edge.information.llt().matrixL();
        // Ceres will take ownership of the pointer.
        ceres::CostFunction* cost_function =
                PoseGraph3dErrorTerm::Create(edge.t_be, sqrt_information);

        problem->AddResidualBlock(cost_function, loss_function,
                                  pose_begin_iter->second.p.data(),
                                  pose_begin_iter->second.q.coeffs().data(),
                                  pose_end_iter->second.p.data(),
                                  pose_end_iter->second.q.coeffs().data());

        problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                     quaternion_local_parameterization);
        problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    MapOfPoses::iterator pose_start_iter = poses->begin();
    problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
    problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem)
{

    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << BOLDCYAN << summary.FullReport() << '\n';

    return summary.IsSolutionUsable();
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const MapOfPoses& poses)
{
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
        cout << "Error opening the file: " << filename;
        return false;
    }
    for (std::map<int, Pose3d, std::less<int>,
         Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
         const_iterator poses_iter = poses.begin();
         poses_iter != poses.end(); ++poses_iter) {
        const std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
                value_type& pair = *poses_iter;
        outfile << pair.first << " " << pair.second.p.transpose() << " "
                << pair.second.q.x() << " " << pair.second.q.y() << " "
                << pair.second.q.z() << " " << pair.second.q.w() << '\n';
    }
    return true;
}

cv::Mat DrawFrameMatch(Frame &currentFrame, Frame &lastFrame)
{
    cv::Mat imMatch = Mat::zeros(currentFrame.mImageGray.rows*2,currentFrame.mImageGray.cols,CV_8UC3);
    vector<cv::KeyPoint> vCurrentKeys = currentFrame.mvKeys;
    vector<cv::KeyPoint> vLastKeys    = lastFrame.mvKeys;
    cv::Mat curr_Im, last_Im;
    curr_Im = currentFrame.mImageGray.clone();
    last_Im = lastFrame.mImageGray.clone();

    if(curr_Im.channels()<3) //this should be always true
    {
        cvtColor(curr_Im,curr_Im,CV_GRAY2BGR);

    }

    if(last_Im.channels()<3) //this should be always true
    {
        cvtColor(last_Im,last_Im,CV_GRAY2BGR);

    }

    /*matches*/
    map<int,int> matches = currentFrame.matchesId;
    const float r = 5;

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt1,pt2,pt3,pt4;
        pt1.x=vCurrentKeys[curr_id].pt.x-r;
        pt1.y=vCurrentKeys[curr_id].pt.y-r;
        pt2.x=vCurrentKeys[curr_id].pt.x+r;
        pt2.y=vCurrentKeys[curr_id].pt.y+r;

        pt3.x=vLastKeys[last_id].pt.x-r;
        pt3.y=vLastKeys[last_id].pt.y-r;
        pt4.x=vLastKeys[last_id].pt.x+r;
        pt4.y=vLastKeys[last_id].pt.y+r;


        //cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im,vCurrentKeys[curr_id].pt,2,cv::Scalar(0,255,0),-1);

        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(0,255,0));
        cv::circle(last_Im,vLastKeys[last_id].pt,2,cv::Scalar(255,0,0),-1);


    }



    curr_Im.copyTo(imMatch(cv::Rect(0,0,curr_Im.cols,curr_Im.rows)));
    last_Im.copyTo(imMatch(cv::Rect(0,curr_Im.rows,curr_Im.cols,curr_Im.rows)));
    /*now we need to draw the lines*/

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt_curr,pt_last;

        pt_curr = vCurrentKeys[curr_id].pt;
        pt_last.x = vLastKeys[last_id].pt.x;
        pt_last.y = vLastKeys[last_id].pt.y + curr_Im.rows;


        cv::line(imMatch, pt_curr, pt_last, cv::Scalar(0,0,255),1,8);


    }
    return imMatch;
}

vector<Mat> DrawReprojectionError(Frame &currentFrame, Frame &lastFrame, cv::Mat &Tcl)
{
    GroundTruth gd;

    cv::Mat mIm = currentFrame.mImageGray.clone();


    /*for matches*/
    cv::Mat imMatch = Mat::zeros(mIm.rows*2,mIm.cols,CV_8UC3);


    cv::Mat imReprojection = Mat::zeros(mIm.rows*2,mIm.cols,CV_8UC3);
    vector<cv::KeyPoint> vCurrentKeys = currentFrame.mvKeys;
    vector<cv::KeyPoint> vLastKeys    = lastFrame.mvKeys;
    vector<float> vDepth = lastFrame.mvDepth; /*depth of last frame*/
    int lastFrameId = lastFrame.mnId;
    int currFrameId = currentFrame.mnId;


//    cv::Mat Tlw = mLastFrame.mTcw;
//    cv::Mat Tcw = mCurrentFrame.mTcw;

//    cv::Mat Tcl = Tcw * Tlw.inv();
    cv::Mat Rcl = Tcl.rowRange(0,3).colRange(0,3);
    cv::Mat tcl = Tcl.rowRange(0,3).col(3);

    /*now we need to load the grodund truth poses to reproject the point*/
    cv::Mat Twl_gd = gd.getFrameTwc(lastFrameId);
    cv::Mat Tcw_gd = gd.getFrameTwc(currFrameId).inv();
    cv::Mat Tcl_gd = Tcw_gd * Twl_gd;
    cv::Mat Rcl_gd = Tcl_gd.rowRange(0,3).colRange(0,3);
    cv::Mat tcl_gd = Tcl_gd.rowRange(0,3).col(3);


    Rcl_gd.convertTo(Rcl_gd,CV_32FC1);
    tcl_gd.convertTo(tcl_gd,CV_32FC1);

    cv::Mat curr_Im, last_Im, curr_Im_match, last_Im_match;



    currentFrame.mImageGray.copyTo(curr_Im);
    lastFrame.mImageGray.copyTo(last_Im);

    if(curr_Im.channels()<3) //this should be always true
    {
        cvtColor(curr_Im,curr_Im,CV_GRAY2BGR);

    }

    if(last_Im.channels()<3) //this should be always true
    {
        cvtColor(last_Im,last_Im,CV_GRAY2BGR);

    }

    curr_Im.copyTo(curr_Im_match);
    last_Im.copyTo(last_Im_match);

    /*matches*/
    map<int,int> matches = currentFrame.matchesId;
    const float r = 5;

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /* since we got the id, we can get the mappoints from last image
         * and we need to reproject it
         */

        /*part2: draw the match*/
        //cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im_match,vCurrentKeys[curr_id].pt,1,cv::Scalar(0,255,0),-1);

        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(0,255,0));
        cv::circle(last_Im_match,vLastKeys[last_id].pt,1,cv::Scalar(255,0,0),-1);

        int      u = vLastKeys[last_id].pt.x;
        int      v = vLastKeys[last_id].pt.y;


        float depth = vDepth[last_id];
        //float depth = 1.0;

        cv::Mat x3Dl = currentFrame.pixel2Camera(u, v, depth);

        /*ground truth*/
        cv::Mat x3Dl_gd = currentFrame.pixel2Camera(u,v,depth);



        /*now we got the 3D point in last frame*/
        /*next we need to get the Tcl. c-->current frame   l--> lastframe*/

        /*before we show it we need to check weather the x3Dl is empty*/
        if(!x3Dl.empty() && !x3Dl_gd.empty())
        {

            cv::Mat x3Dc = Rcl * x3Dl + tcl;
            cv::Mat x3Dc_gd = Rcl_gd * x3Dl_gd + tcl_gd;

            /*OK, let's reproject this point*/

            Point2f pixel = currentFrame.camera2Pixel(x3Dc);
            Point2f pixel_gd = currentFrame.camera2Pixel(x3Dc_gd);

            /*now we have reproject the point which in last frame to the current frame*/

            /*let's show it and draw a line between the current match features*/

            cv::circle(curr_Im,pixel,1,cv::Scalar(255,0,0),-1);
            cv::circle(curr_Im,pixel_gd,1,cv::Scalar(0,0,255),-1);

            cv::line(curr_Im,vCurrentKeys[curr_id].pt, pixel, Scalar(255,0,0),1);
            cv::line(curr_Im,vCurrentKeys[curr_id].pt, pixel_gd, Scalar(0,0,255),1);
        }



        /*draw the current frame, the first value*/
        cv::Point2f pt1,pt2,pt3,pt4;
        pt1.x=vCurrentKeys[curr_id].pt.x-r;
        pt1.y=vCurrentKeys[curr_id].pt.y-r;
        pt2.x=vCurrentKeys[curr_id].pt.x+r;
        pt2.y=vCurrentKeys[curr_id].pt.y+r;

        pt3.x=vLastKeys[last_id].pt.x-r;
        pt3.y=vLastKeys[last_id].pt.y-r;
        pt4.x=vLastKeys[last_id].pt.x+r;
        pt4.y=vLastKeys[last_id].pt.y+r;




        //cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im,vCurrentKeys[curr_id].pt,1,cv::Scalar(0,255,0),-1);


        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(255,0,0));
        cv::circle(last_Im,vLastKeys[last_id].pt,1,cv::Scalar(255,0,0),-1);

        /*show the depth*/
        stringstream s;
        s << depth;
        cv::putText(last_Im, s.str(), vLastKeys[last_id].pt, cv::FONT_HERSHEY_PLAIN,0.5,cv::Scalar(255,255,255),1,8 );


    }

    curr_Im_match.copyTo(imMatch(cv::Rect(0,0,curr_Im_match.cols,curr_Im_match.rows)));
    last_Im_match.copyTo(imMatch(cv::Rect(0,curr_Im_match.rows,curr_Im_match.cols,curr_Im_match.rows)));
    /*prat2:now we need to draw the lines*/
    /*now we need to draw the lines*/

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt_curr,pt_last;

        pt_curr = vCurrentKeys[curr_id].pt;
        pt_last.x = vLastKeys[last_id].pt.x;
        pt_last.y = vLastKeys[last_id].pt.y + curr_Im.rows;


        cv::line(imMatch, pt_curr, pt_last, cv::Scalar(0,0,255),1,8);


    }




    curr_Im.copyTo(imReprojection(cv::Rect(0,0,curr_Im.cols,curr_Im.rows)));
    last_Im.copyTo(imReprojection(cv::Rect(0,curr_Im.rows,curr_Im.cols,curr_Im.rows)));


//    stringstream ss1;
//    ss1 << currFrameId;
//    string tmp;
//    ss1 >> tmp;
//    string matchsName = "/home/m/ws_orb2/src/ORB_SLAM2/Matches/" + tmp + ".png";
//    //imwrite(matchsName, imMatch);
//    cout << BOLDCYAN"write " << tmp << " suscessfully!" << endl;

    //resize(imReprojection,imReprojection,Size(),0.7,0.8);
//    stringstream ss;
//    ss << currFrameId;

//    string filename = "/home/m/ws_orb2/src/ORB_SLAM2/Reprojection/" + tmp + ".png";
//    //imwrite(filename,imReprojection);

    vector<Mat> output;
    output.push_back(imReprojection);
    output.push_back(imMatch);

    return output;


}




