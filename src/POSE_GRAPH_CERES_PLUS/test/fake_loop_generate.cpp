#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "camera.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"
#include "ImageRead.h"

#include <fstream>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace POSE_GRAPH;

int main(int argc, char *argv[])
{

    Config::setParameterFile("../config/config.yaml");
    string dir = Config::get<string>("sequence_dir");

    ifstream inFile;
    //inFile.open("/home/m/KITTI/dataset/sequences/99/trajectory.txt");
    ofstream outFile;
    outFile.open("/home/m/KITTI/dataset/sequences/99/trajectory1.txt");
    vector<int> Quality;
    Quality.reserve(1);
    Quality.push_back(0);


    GroundTruth gd;


    for(int32_t i = 0; i < 4541; i++)
    {

        cout << "process: " << i << endl;
        cv::Mat Twc = gd.getFrameTwc(i);
        Pose3d pose = Converter::toPose3d(Twc);
        Eigen::Vector3d p = pose.p;
        Eigen::Quaterniond q = pose.q;

        srand((unsigned)time(NULL));



        if(i > 4451)
        {
            /*fake_loop*/
            int xdelta = (rand() % (3-0))+ 0;
            p(0) = p(0)+ (double)xdelta;

            outFile << p(0)  << " " << p(1) << " " << p(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
            outFile << p(0)  << " " << p(1) << " " << p(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        cv::waitKey(27);

    }


    return 0;
}


