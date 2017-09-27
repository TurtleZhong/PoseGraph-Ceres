#include "GroundTruth.h"
#include "config.h"
#include "string.h"
#include "converter.h"
#include <fstream>

namespace POSE_GRAPH
{
GroundTruth::GroundTruth()
{
    int fileFormat = Config::get<int> ("trajectory_format");
//    cout << "fileFormat = " << fileFormat << endl;

    if(fileFormat == 1)
        this->loadPoses1();
    else if (fileFormat == 2)
        this->loadPoses2();
    else
        cout << "Invalid trajectory format! Please check the file format." << endl;
}

void GroundTruth::loadPoses2()
{
    this->sequenceDir = Config::get< string > ("poses_dir");
    fstream inFile;

    inFile.open(sequenceDir.c_str());
    while(inFile.good())
    {
        Mat P = Mat::eye(4,4,CV_64FC1);

        for (int row = 0; row <3; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                inFile >> P.at<double>(row,col);
            }
        }
        P.convertTo(P,CV_32FC1);
        cv::Mat tmp(P.rows, P.cols, CV_32FC1);
        tmp = P.clone();
        poses.push_back(tmp);
        inFile.get();
    }

}

void GroundTruth::loadPoses1()
{
    this->sequenceDir = Config::get< string > ("poses_dir");
    fstream inFile;

    inFile.open(sequenceDir.c_str());
    while(inFile.good())
    {

        Vector3d p;

        double trans[4] = {0,0,0,0};

        inFile >> p(0) >> p(1) >> p(2) >> trans[1] >> trans[2] >> trans[3] >> trans[0];
        Eigen::Quaterniond q(trans);

        Pose3d pose;
        pose.p = p;
        pose.q = q;

        cv::Mat P = Converter::toCvMat(pose);
        poses.push_back(P);
        inFile.get();
    }

}

Mat GroundTruth::getFrameTwc(int frameID)
{

    if (frameID < (int)poses.size())
    {
        return poses[frameID];

    }

    else
        return Mat();
}

Mat GroundTruth::getFrameRwc(int frameID)
{

    if (frameID < (int)poses.size())
    {
        return Mat(poses[frameID]).rowRange(0,3).colRange(0,3);
    }

    else
        return Mat();
}

Mat GroundTruth::getFrametwc(int frameID)
{

    if (frameID < (int)poses.size())
    {
        return Mat(poses[frameID]).rowRange(0,3).col(3);
    }

    else
        return Mat();
}

Mat GroundTruth::getFrameRcw(int frameID)
{

    if (frameID < (int)poses.size())
    {
        return Mat(getFrameRwc(frameID)).t();
    }

    else
        return Mat();
}

Mat GroundTruth::getFrametcw(int frameID)
{

    if (frameID < (int)poses.size())
    {
        return Mat(-1*getFrameRcw(frameID))* Mat(getFrametwc(frameID));
    }

    else
        return Mat();
}

Mat GroundTruth::getFrameTcw(int frameID)
{
    if (frameID < (int)poses.size())
    {
        cv::Mat Tcw = Mat::eye(4,4,CV_32FC1);
        cv::Mat Rcw = getFrameRcw(frameID);
        cv::Mat tcw = getFrametcw(frameID);

        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(Tcw.rowRange(0,3).col(3));

        return Tcw;
    }
    else
        return Mat();
}

}


