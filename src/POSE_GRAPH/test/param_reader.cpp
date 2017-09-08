#include "common_include.h"
#include "config.h"
#include "GroundTruth.h"

using namespace POSE_GRAPH;

int main(int argc, char *argv[])
{
    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH/config/config.yaml");

    GroundTruth gd;
    cout << "gd.getFrameTwc(0) = \n" << gd.getFrameTwc(0) << endl;
    cv::Mat T = gd.getFrameTwc(0);
    T.convertTo(T,CV_64FC1);
    cout << "T = \n" << T <<  T.type() << endl;

    return 0;
}
