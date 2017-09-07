#include "common_include.h"
#include "config.h"
#include "GroundTruth.h"

int main(int argc, char *argv[])
{
    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH/config/config.yaml");

    GroundTruth gd;
    cout << "gd.getFrameTwc(0) = \n" << gd.getFrameTwc(0) << endl;

    return 0;
}
