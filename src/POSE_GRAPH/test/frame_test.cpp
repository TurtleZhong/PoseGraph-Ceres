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

    SequenceRun* sequenceRun = new SequenceRun();
    Frame lastFrame;

    for(int32_t i = 0; i < 4541; i++)
    {
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        cout << "Process :" << i << endl;

        sequenceRun->GrabImageStereo(Left,Right,0.0);
        Frame currentFrame = sequenceRun->mCurrentFrame;
        cout << "keypoints.size = " << currentFrame.mvKeysUn.size() << endl;

        const std::vector<cv::KeyPoint> &keypoints = currentFrame.mvKeys;

//        for(std::vector<cv::KeyPoint>::const_iterator vkeys = keypoints.begin(); vkeys!= keypoints.end(); vkeys++)
//        {
//            cout << BOLDGREEN"keypoints.u = " << (*vkeys).pt.x <<  " keypoints.v = " << (*vkeys).pt.y << endl;
//        }

        cout << "currentFrame.id = " << currentFrame.mnId << endl;

        cv::imshow("currentFrame", sequenceRun->mImGray);

        if(i > 2)
        {
            cv::Mat deltaT = currentFrame.mTcw * lastFrame.mTwc; //-->Tcl
            cv::Mat t = deltaT.rowRange(0,3).col(3);
            cv::Mat t1 = Mat::zeros(t.rows,t.cols,t.type());
            cout << "delta T = \n" << deltaT << endl;
            //double dist = cvNorm(t, NORM_L2 );
            //cout << "norm L2 = " << dist << endl;
        }




        cv::waitKey(27);

        lastFrame = currentFrame;


    }


    return 0;
}


