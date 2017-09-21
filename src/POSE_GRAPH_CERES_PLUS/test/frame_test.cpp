#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "camera.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"

#include <fstream>

using namespace std;
using namespace POSE_GRAPH;

int main(int argc, char *argv[])
{

    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH/config/config.yaml");
    string dir = Config::get<string>("sequence_dir");

    ifstream inFile;
    //inFile.open("/home/m/KITTI/dataset/sequences/99/timestamp.txt");
    ofstream outFile;
    outFile.open("/home/m/KITTI/dataset/sequences/99/trajectory.txt");
    vector<int> Quality;
    Quality.reserve(1);
    Quality.push_back(0);


    GroundTruth gd;


    for(int32_t i = 0; i < 4541; i++)
    {
//        char base_name[256];
//        sprintf(base_name,"%06d.png",i);
//        string left_img_file_name = dir + "/image_0/" + base_name;
//        string right_img_file_name = dir + "/image_1/" + base_name;
//        Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
//        Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
//        string ss;
//        inFile >> ss;
//        ss = ss + ".png";
//        string sstmp = ss;
//        ss = "/home/m/KITTI/dataset/sequences/99/image_0/" + ss;
//        string ss1;
//        ss1 = "/home/m/KITTI/dataset/sequences/99/image_1/" + sstmp;
        cout << "process: " << i << endl;

//        imwrite(ss,Left,Quality);
//        imwrite(ss1,Right,Quality);
        cv::Mat Twc = gd.getFrameTwc(i);
        Pose3d pose = Converter::toPose3d(Twc);
        Eigen::Vector3d p = pose.p;
        Eigen::Quaterniond q = pose.q;

        outFile << p(0)  << " " << p(1) << " " << p(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        cv::waitKey(27);



    }


    return 0;
}


