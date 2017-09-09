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
void checkFrame(Frame &frame1, Frame &frame2, g2o::SparseOptimizer &optimizer);
std::vector<Frame> getCandidateFrames(vector<Frame>& vFrames, Frame &currentFrame);

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
    optimizer.optimize( 100 );
    optimizer.save( "../result/g2o/result_after.g2o" );
    cout<<"Optimization done."<<endl;






    return 0;
}

void checkForPoseGraph(vector<Frame> &vFrames, Frame &currentFrame, g2o::SparseOptimizer &optimizer)
{
    for(size_t i = 0; i < vFrames.size(); i++)
    {
        checkFrame(vFrames[i], currentFrame, optimizer);
    }
}

void checkFrame(Frame &frame1, Frame &frame2, g2o::SparseOptimizer &optimizer)
{
    /*first we need to check this two frames --> use ORBmatcher*/

    ORBmatcher matcher(0.7,true);

    int nmatches = matcher.SearchByProjection(frame2,frame1,5,false);
    cout << "matches = " << nmatches << " ";

    if(frame2.mnId - frame1.mnId == 1)
    {
        // EDGE
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();


        edge->setVertex( 0, optimizer.vertex(frame1.mnId ));
        edge->setVertex( 1, optimizer.vertex(frame2.mnId ));
        edge->setRobustKernel( new g2o::RobustKernelHuber() );

        // imformation Matrix
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;

        edge->setInformation( information );
        Eigen::Isometry3d T = Converter::toIsometry3d(frame2.mTcw * frame1.mTwc);
        edge->setMeasurement( T );
        optimizer.addEdge(edge);
    }

    else if(nmatches > 300)
    {
        // EDGE
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();


        edge->setVertex( 0, optimizer.vertex(frame1.mnId ));
        edge->setVertex( 1, optimizer.vertex(frame2.mnId ));
        edge->setRobustKernel( new g2o::RobustKernelHuber() );

        // imformation Matrix
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
//        information(0,0) = information(1,1) = information(2,2) = 100;
//        information(3,3) = information(4,4) = information(5,5) = 100;

        edge->setInformation( information );
        Eigen::Isometry3d T = Converter::toIsometry3d(frame2.mTcw * frame1.mTwc);
        edge->setMeasurement( T );
        optimizer.addEdge(edge);

        cout << BOLDYELLOW"add a new edge! " << frame2.mnId << " --> " << frame1.mnId <<  endl;
    }
    cout << endl;

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
