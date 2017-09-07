#include "common_include.h"
#include "config.h"
#include "tracking.h"
#include "viso_stereo.h"
#include "camera.h"
#include "frame.h"
#include "fstream"

//g2o parts


int main(int argc, char *argv[])
{
    /*write file parts*/
    ofstream outFile;
    outFile.open("../camera_pose.txt");


    string dir = "/home/m/KITTI/dataset/sequences/00";
    Config::setParameterFile("/home/m/ws_orb2/src/VISO2+/config/KITTI00-02.yaml");
    Camera* camera (new Camera);


    Tracking* Tracker = new Tracking();
    Frame currentFrame,lastFrame;

    for(int32_t i = 0; i < 4541; i++)
    {
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);

        //Frame currentFrame(i,camera,Left,Right);
        cout << "Process :" << i << endl;
        Tracker->generateFrame(i,camera,Left,Right);


        //Tracker->addFrame(i,camera,Left,Right);
        currentFrame = Frame(Tracker->curr_);
        std::vector<Matcher::p_match> &matched = currentFrame.mvStereoMatches;
        /*generate the pair*/
        std::vector<cv::Point2f> points1,points2;
        for(i = 0; i < matched.size(); i++)
        {
            cv::Point2f pt1,pt2;
            pt1.x = matched[i].u1c;
            pt1.y = matched[i].v1c;
            pt2.x = matched[i].u1p;
            pt2.y = matched[i].v1p;
            points1.push_back(pt1);
            points2.push_back(pt2);
        }



        Mat pose = currentFrame.mTcw.inv();

        cout << "currentframe.mTcw = " << pose << endl;
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(i==2 && j==3)
                    outFile << pose.at<double>(i,j);
                else
                    outFile << pose.at<double>(i,j) << " ";

            }
        }
        outFile << endl;

        lastFrame = currentFrame;
    }
    outFile.close();


    return 0;
}

