#ifndef SEQUENCERUN_H
#define SEQUENCERUN_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include "Frame.h"
#include "ORBextractor.h"
#include "GroundTruth.h"



namespace POSE_GRAPH
{

class SequenceRun
{
public:
    // constructor
    SequenceRun();
    bool GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);
    bool GenerateFrameMappoints();

public:
    Frame mCurrentFrame;
    cv::Mat mImGray;

protected:
    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    //Ground Truth
    GroundTruth gd;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

};

}


#endif // SEQUENCERUN_H
