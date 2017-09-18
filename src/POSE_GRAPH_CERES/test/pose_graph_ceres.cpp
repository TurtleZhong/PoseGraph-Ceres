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
int findFeatureMatches(Frame &lastFrame, Frame &currentFrame, vector<DMatch> &matches, string method = "BF");
RESULT_OF_PNP motionEstimate(Frame &frame1, Frame &frame2);
void BuildOptimizationProblem(const VectorOfEdges& Edges,
                              MapOfPoses* poses, ceres::Problem* problem);
bool SolveOptimizationProblem(ceres::Problem* problem);
bool OutputPoses(const std::string& filename, const MapOfPoses& poses);


int main(int argc, char *argv[])
{


    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH_CERES/config/config08.yaml");
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

    for(int32_t i = 0; i < sequenceLength; i++)
    {
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        cout << RESET"Process :" << i << endl;

        sequenceRun->GrabImageStereo(Left,Right,0.0);
        Frame currentFrame = sequenceRun->mCurrentFrame;

        /*add the frame vertex*/

        Pose3d currentPose =Converter::toPose3d(currentFrame.mTwc);
        poses[currentFrame.mnId] = currentPose;

        /*the first edge*/
        if(i == 0)
        {
            Edge3d edge;
            edge.id_begin = 0;
            edge.id_end = 0;
            cv::Mat Tcl = cv::Mat::eye(4,4,CV_32F);
            Pose3d T = Converter::toPose3d(Tcl);
            edge.t_be = T;
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
            edge.information = information;
            Edges.push_back(edge);
        }



        /*add vertex and edges to generate pose graph*/
        if(i > 0)
        {
            vector<Frame> vCandidateFrames = getCandidateFrames(vFrames,currentFrame);

            checkForPoseGraph(vCandidateFrames, currentFrame, Edges);
        }

        //cv::imshow("currentFrame", sequenceRun->mImGray);


        //cv::waitKey(27);

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

    /*after the data were generated, we created the pose_graph */
    BuildOptimizationProblem(Edges,poses,problem);
    bool isSuscess = SolveOptimizationProblem(problem);
    if(isSuscess)
        cout << "Optimizing Suscessfully!" << endl;
    else
        cout << "May be some problems!" << endl;

    // Output the poses to the file with format: id x y z q_x q_y q_z q_w.
    OutputPoses("/home/m/ws_orb2/src/POSE_GRAPH_CERES/pose_graph.txt",poses);


    return 0;
}

void checkForPoseGraph(vector<Frame> &vFrames, Frame &currentFrame, Edge3d &Edges)
{
    for(size_t i = 0; i < vFrames.size(); i++)
    {
        checkFrame(vFrames[i], currentFrame, Edges);
    }


}

void checkFrame(Frame &frame1, Frame &frame2, VectorOfEdges &Edges)
{
    /*first we need to check this two frames --> use ORBmatcher*/

    ORBmatcher matcher(0.7,true);

    int nmatches = matcher.MatcheTwoFrames(frame2,frame1,5,false);
    //vector<DMatch> matches;
    //int nmatches = findFeatureMatches(frame1,frame2,matches);
    cout << "matches = " << nmatches << " ";

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


    }

    else if(nmatches > 180)
    {

        /*now we check the motion and inliers of this two frame*/
        RESULT_OF_PNP result = motionEstimate(frame1,frame2);
        double norm = normofTransform(result.rvec,result.tvec);

        if(result.inliers > 80 && norm < 1.2)
        {
            Edge3d edge;
            edge.id_begin = frame2.mnId;
            edge.id_end = frame1.mnId;
            cv::Mat Tcl = frame2.mTcw*frame1.mTwc;
            Pose3d T = Converter::toPose3d(Tcl);
            edge.t_be = T;
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
            edge.information = information;

            Edges.push_back(edge);

            cout << BOLDYELLOW"add a new edge! " << frame2.mnId << " --> " << frame1.mnId <<  endl;
        }

    }
    cout << endl;

}

std::vector<Frame> getCandidateFrames(vector<Frame>& vFrames, Frame &currentFrame)
{
    /*here we need to get the candidate frames according to the Twc*/
    std::vector<Frame> candidates;
    //cout << BOLDGREEN"candidate frames id = ";

    for(vector<Frame>::const_iterator vit = vFrames.begin(); vit!=vFrames.end();vit++)
    {
        if(currentFrame.mnId - Frame(*vit).mnId < 100)
            continue;

        cv::Mat twc = Frame(*vit).GetCameraCenter();

        if(currentFrame.mnId - Frame(*vit).mnId < 5) /*near by frames --> can write to the config files*/
        {
            candidates.push_back(*vit);
        }
        bool isInRange = currentFrame.isInSearchRange(twc);
        if(isInRange)
        {
            if(currentFrame.mnId - Frame(*vit).mnId >= 100 )
            {
                candidates.push_back(*vit);
                cout << Frame(*vit).mnId << " ";
                cout << BOLDRED"may be we got a loop!" << endl;
            }

        }

    }

    cout << endl;

    return candidates;

}

int findFeatureMatches(Frame &lastFrame,
                       Frame &currentFrame,
                       vector<DMatch> &matches,
                       string method)
{
    /*
      * define the detect param: use ORB
      */
    Ptr<ORB> orb = ORB::create(1000);
    Mat descriptors;
    std::vector<KeyPoint> keypoints1,keypoints2;
    /*
      * use detect() function to detect keypoints
      */
    orb-> detect(currentFrame.mImageGray, keypoints1);
    /*
      * conpute the extractor and show the keypoints
      */
    orb-> compute(currentFrame.mImageGray, keypoints1, descriptors);

    Mat testDescriptors;
    orb->detect(lastFrame.mImageGray, keypoints2);
    orb->compute(lastFrame.mImageGray, keypoints2,testDescriptors);

    //vector<DMatch> match;
    /*
      * FLANN
      */

    if(method == "KNN" || method == "knn")
    {
        testDescriptors.convertTo(testDescriptors,CV_32S);
        descriptors.convertTo(testDescriptors,CV_32S);
        flann::Index flannIndex(testDescriptors, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);



        /*Match the feature*/
        Mat matchIndex(descriptors.rows, 2, CV_32S);
        Mat matchDistance(descriptors.rows, 2, CV_32S);
        flannIndex.knnSearch(descriptors.rows, matchIndex, matchDistance, 2, flann::SearchParams());

        //vector<DMatch> goodMatches;
        for (int i = 0; i < matchDistance.rows; i++)
        {
            if(matchDistance.at<float>(i,0) < 0.75 * matchDistance.at<float>(i, 1))
            {
                DMatch dmatchs(i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,1));
                matches.push_back(dmatchs);
            }
        }

    }
    else if(method == "BF")
    {
        vector<DMatch> tmpMatches;
        BFMatcher matcher(NORM_HAMMING);
        matcher.match(descriptors, testDescriptors, tmpMatches);
        double min_dist = 10000, max_dist = 0;
        for(int i = 0; i < descriptors.rows; i++)
        {
            double dist = tmpMatches[i].distance;
            if(dist < min_dist) min_dist = dist;
            if(dist > max_dist) max_dist = dist;
        }
        cout << "--Max dist = " << max_dist << endl;
        cout << "--Min dist = " << min_dist << endl;

        for(int i =0; i < descriptors.rows; i++)
        {
            if(tmpMatches[i].distance <= max(2*min_dist, 30.0))
            {
                matches.push_back(tmpMatches[i]);
            }
        }

    }

    Mat resultImage;
    drawMatches(currentFrame.mImageGray, keypoints1, lastFrame.mImageGray, keypoints2, matches, resultImage);
    imshow("result of Image", resultImage);
    cv::waitKey(27);

    cout << "We got " << matches.size() << " good Matchs" << endl;
    return (int)matches.size();
}

/*we need a simple function to estimate the inliers and pose*/
RESULT_OF_PNP motionEstimate(Frame &frame1, Frame &frame2)
{
    RESULT_OF_PNP result;
    map<int,int> matches = frame2.matchesId;
    // 3d points in first frame
    vector<cv::Point3f> pts_obj;
    // pixel in second frame
    vector< cv::Point2f > pts_img;

    // Camera internal params
    Camera* pCamera = new Camera();

    for(map<int,int>::const_iterator mit = matches.begin(); mit!=matches.end(); mit++)
    {

        cv::Point2f p = frame1.mvKeys[mit->second].pt;
        pts_img.push_back( cv::Point2f( frame2.mvKeys[mit->first].pt ) );

        // Transform (u,v,d) to (x,y,z)
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

    // construct the camera Matrix
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // Solve pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 200, 4.0, 0.99, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

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
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

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
