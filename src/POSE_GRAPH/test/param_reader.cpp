#include "common_include.h"
#include "config.h"
#include "GroundTruth.h"

using namespace POSE_GRAPH;

int main(int argc, char *argv[])
{
    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH/config/config.yaml");

    GroundTruth gd;
    cout << "gd.getFrameTwc(0) = \n" << gd.getFrameTwc(0) << endl;
    cv::Mat T = gd.getFrameTcw(32);
    cv::Mat T1 = gd.getFrameTwc(31);
    cv::Mat result = T * T1;
    cout << "T = \n" << T <<  T.type() << endl;
    cout << "T* T' =\n " << T * T.inv() << endl;
    cout << "T1 = \n" << T1.inv() << endl;
    cout << "T1 = \n" << gd.getFrameTcw(828) << endl;
    cout << "result = \n" << result << endl;
    return 0;
}
