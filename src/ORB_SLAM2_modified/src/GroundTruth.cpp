#include "GroundTruth.h"

namespace ORB_SLAM2 {

GroundTruth::GroundTruth()
{

    this->loadPoses();
}

void GroundTruth::loadPoses()
{
    //this->sequenceDir = "/home/m/KITTI/poses/00.txt";
    this->sequenceDir = "/home/m/KITTI/devkit/cpp/results/ORB/data/11.txt";
    //
    fstream inFile;
    inFile.open(sequenceDir);
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
        poses.push_back(P);
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

}
