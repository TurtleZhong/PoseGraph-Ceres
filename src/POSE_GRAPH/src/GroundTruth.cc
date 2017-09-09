#include "GroundTruth.h"
#include "config.h"
#include "string.h"
#include <fstream>

namespace POSE_GRAPH
{
GroundTruth::GroundTruth()
{

    this->loadPoses();
}

void GroundTruth::loadPoses()
{
    /*"/home/m/KITTI/poses/00.txt"*/
    this->sequenceDir = Config::get< string > ("poses_dir");
    fstream inFile;

    inFile.open(sequenceDir.c_str());
    int count = 0;
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
        count++;
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


