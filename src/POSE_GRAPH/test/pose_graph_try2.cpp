#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "camera.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"
#include "ORBmatcher.h"

/*g2o parts*/
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


#include <fstream>

using namespace std;
using namespace POSE_GRAPH;


void checkForPoseGraph(vector<Frame>& vFrames, Frame &currentFrame, g2o::SparseOptimizer &optimizer);
int checkFrame(Frame &frame1, Frame &frame2);
std::vector<Frame> getCandidateFrames(vector<Frame>& vFrames, Frame &currentFrame);
int findFeatureMatches(Frame &currentFrame, Frame &lastFrame, string method = "BF");

int main(int argc, char *argv[])
{



    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH/config/config.yaml");
    string dir = Config::get<string>("sequence_dir");

    /*g2o parts*/

    g2o::SparseOptimizer optimizer;
    g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    linearSolver->setBlockOrdering(false);

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);


    // Set Frame vertex --> first frame
    //    g2o::VertexSE3 * vSE3 = new g2o::VertexSE3();
    //    vSE3->setEstimate(Eigen::Isometry3d::Identity());
    //    vSE3->setId(0);
    //    vSE3->setFixed(true);
    //    optimizer.addVertex(vSE3);


    SequenceRun* sequenceRun = new SequenceRun();
    vector<Frame> vFrames;

    ofstream outFile;
    outFile.open("../camera_poses.txt");

    for(int32_t i = 0; i < 2761; i++)
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
        g2o::VertexSE3 * vSE3 = new g2o::VertexSE3();
        vSE3->setEstimate(Eigen::Isometry3d::Identity());
        vSE3->setId(currentFrame.mnId);
        optimizer.addVertex(vSE3);
        if(currentFrame.mnId == 0)
            vSE3->setFixed(true);
        else
            vSE3->setFixed(false);


        /*add vertex and edges to generate pose graph*/
        if(i > 0)
        {
            vector<Frame> vCandidateFrames = getCandidateFrames(vFrames,currentFrame);

            checkForPoseGraph(vCandidateFrames, currentFrame, optimizer);
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

    cout<<RESET"optimizing pose graph, vertices: "<<optimizer.vertices().size()<<endl;
    optimizer.save("../result/g2o/result_before.g2o");
    optimizer.initializeOptimization();
    optimizer.optimize( 10000 );
    optimizer.save( "../result/g2o/result_after.g2o" );
    cout<<"Optimization done."<<endl;






    return 0;
}

void checkForPoseGraph(vector<Frame> &vFrames, Frame &currentFrame, g2o::SparseOptimizer &optimizer)
{
    map<int,int> mid_value;
    for(size_t i = 0; i < vFrames.size() - 1; i++)
    {
        int matches = checkFrame(vFrames[i], currentFrame);
        if(matches > 400)
            mid_value.insert(pair<int,int>(i, matches));
    }

    vector< pair<int,int> > sortResult;

    if(mid_value.size() >= 3 )
    {
        for(map<int,int>::iterator iter = mid_value.begin(); iter!=mid_value.end(); iter++)
        {
            sortResult.push_back(make_pair(iter->first, iter->second));
        }

        sort(sortResult.begin(),sortResult.end());


        /*actually we need to add the back three*/

        for(int i = mid_value.size() - 2; i < mid_value.size(); i++)
        {
            // EDGE
            g2o::EdgeSE3* edge = new g2o::EdgeSE3();


            edge->setVertex( 0, optimizer.vertex(vFrames[sortResult[i].first].mnId ));
            edge->setVertex( 1, optimizer.vertex(currentFrame.mnId ));
            edge->setRobustKernel( new g2o::RobustKernelHuber() );

            // imformation Matrix
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
            //        information(0,0) = information(1,1) = information(2,2) = 100;
            //        information(3,3) = information(4,4) = information(5,5) = 100;

            edge->setInformation( information );
            Eigen::Isometry3d T = Converter::toIsometry3d(currentFrame.mTcw * vFrames[sortResult[i].first].mTwc);
            edge->setMeasurement( T );
            optimizer.addEdge(edge);

            cout << BOLDYELLOW"add a new edge! " << currentFrame.mnId << " --> " << vFrames[sortResult[i].first].mnId <<  endl;
        }

    }



    // EDGE
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();


    edge->setVertex( 0, optimizer.vertex(vFrames.back().mnId ));
    edge->setVertex( 1, optimizer.vertex(currentFrame.mnId ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );

    // imformation Matrix
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    //        information(0,0) = information(1,1) = information(2,2) = 100;
    //        information(3,3) = information(4,4) = information(5,5) = 100;

    edge->setInformation( information );
    Eigen::Isometry3d T = Converter::toIsometry3d(currentFrame.mTcw * vFrames.back().mTwc);
    edge->setMeasurement( T );
    optimizer.addEdge(edge);




}

int checkFrame(Frame &frame1, Frame &frame2)
{
    /*first we need to check this two frames --> use ORBmatcher*/

    int nmatches = findFeatureMatches(frame2,frame1);
    cout << "matches = " << nmatches << " ";
    return nmatches;
}

std::vector<Frame> getCandidateFrames(vector<Frame>& vFrames, Frame &currentFrame)
{
    /*here we need to get the candidate frames according to the Twc*/
    std::vector<Frame> candidates;
    cout << BOLDGREEN"candidate frames id = ";

    for(vector<Frame>::const_iterator vit = vFrames.begin(); vit!=vFrames.end();vit++)
    {
        cv::Mat twc = Frame(*vit).GetCameraCenter();

        if(vit == vFrames.end() - 1)
        {
            candidates.push_back(*vit);
        }
        bool isInRange = currentFrame.isInSearchRange(twc);
        if(isInRange)
        {
            if(currentFrame.mnId - Frame(*vit).mnId > 70 )
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

int findFeatureMatches(Frame &currentFrame,
                       Frame &lastFrame,
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

    vector<DMatch> matches;
    /*
      * FLANN
      */

    if(method == "KNN" || method == "knn")
    {
        flann::Index flannIndex(testDescriptors, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);



        /*Match the feature*/
        Mat matchIndex(descriptors.rows, 2, CV_32S);
        Mat matchDistance(descriptors.rows, 2, CV_32S);
        flannIndex.knnSearch(descriptors.rows, matchIndex, matchDistance, 2, flann::SearchParams());

        //vector<DMatch> goodMatches;
        for (int i = 0; i < matchDistance.rows; i++)
        {
            if(matchDistance.at<float>(i,0) < 0.7 * matchDistance.at<float>(i, 1))
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
