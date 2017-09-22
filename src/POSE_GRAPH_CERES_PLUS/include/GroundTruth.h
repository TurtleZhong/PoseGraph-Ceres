#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H


#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>


using namespace std;
using namespace cv;

namespace POSE_GRAPH
{
class GroundTruth
{
public:
    GroundTruth();


    Mat getFrameTwc(int frameID);
    Mat getFrameRwc(int frameID);
    Mat getFrametwc(int frameID);
    Mat getFrameRcw(int frameID);
    Mat getFrametcw(int frameID);
    Mat getFrameTcw(int frameID);
    void loadPoses1();
    void loadPoses2();

private:
    vector <cv::Mat> poses;
    string sequenceDir;

};
}





#endif // GROUNDTRUTH_H
