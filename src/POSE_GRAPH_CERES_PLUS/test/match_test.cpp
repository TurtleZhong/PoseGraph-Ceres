#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "camera.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"
#include "ORBmatcher.h"
#include <opencv2/opencv.hpp>

#include <fstream>

using namespace std;
using namespace POSE_GRAPH;

//int find_feature_matches(Frame &currentFrame,
//                         Frame &lastFrame,
//                         vector<DMatch>& matches,
//                         string method = "BF");
struct RESULT_OF_PNP
{
    cv::Mat rvec;
    cv::Mat tvec;
    int inliers;
};
cv::Mat DrawFrameMatch(Frame &currentFrame, Frame &lastFrame);
RESULT_OF_PNP motionEstimate(Frame &frame1, Frame &frame2);
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main(int argc, char *argv[])
{

    Config::setParameterFile("../config/config00.yaml");
    string dir = Config::get<string>("sequence_dir");

    SequenceRun* sequenceRun = new SequenceRun();
    Frame currentFrame, lastFrame;
    GroundTruth gd;

    for(int32_t i = 0; i < 2761; i = i+1)
    {
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        cout << "Process :" << i << endl;

        sequenceRun->GrabImageStereo(Left,Right,0.0);
        currentFrame = sequenceRun->mCurrentFrame;

        cv::Mat twc = gd.getFrametwc(i);
        cout << "twc = " << endl << twc << endl;

        /*Do the feature matching test*/

        ORBmatcher matcher(0.7, false);

        if(i > 3)
        {
            int nmatches = matcher.MatcheTwoFrames(currentFrame,lastFrame,false);
            cout << "we got " << nmatches << " matches!" << endl;

            RESULT_OF_PNP result;
            result = motionEstimate(lastFrame,currentFrame);

            DrawFrameMatch(currentFrame,lastFrame);

            cout << "tvec = \n" << result.tvec << endl;
            cout << "inliers = " << result.inliers << endl;

            /*groundTruth*/
            cv::Mat Tcl = currentFrame.mTcw * lastFrame.mTwc;
            cv::Mat tcl = Tcl.rowRange(0,3).col(3);
            cout << "GroundTruth tcl = " << endl<< tcl << endl;

            double norm = normofTransform(result.rvec, result.tvec);
            cout << "norm = " << norm << endl;

            Vector3d p(1,2,3);
//            double* pdata = p.data();
//            cout << pdata[0] << pdata[1] << pdata[2] << endl;
//            cout << *pdata << *(pdata+1) << *(pdata+2) << endl;

//            double* y = pdata+1;
//            cout << *y << endl;





        }

        cv::imshow("currentFrame", sequenceRun->mImGray);


        cv::waitKey(27);

        lastFrame = currentFrame;

    }


    return 0;
}


cv::Mat DrawFrameMatch(Frame &currentFrame, Frame &lastFrame)
{
    cv::Mat imMatch = Mat::zeros(currentFrame.mImageGray.rows*2,currentFrame.mImageGray.cols,CV_8UC3);
    vector<cv::KeyPoint> vCurrentKeys = currentFrame.mvKeys;
    vector<cv::KeyPoint> vLastKeys    = lastFrame.mvKeys;
    cv::Mat curr_Im, last_Im;
    curr_Im = currentFrame.mImageGray.clone();
    last_Im = lastFrame.mImageGray.clone();

    if(curr_Im.channels()<3) //this should be always true
    {
        cvtColor(curr_Im,curr_Im,CV_GRAY2BGR);

    }

    if(last_Im.channels()<3) //this should be always true
    {
        cvtColor(last_Im,last_Im,CV_GRAY2BGR);

    }

    /*matches*/
    map<int,int> matches = currentFrame.matchesId;
    const float r = 5;

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt1,pt2,pt3,pt4;
        pt1.x=vCurrentKeys[curr_id].pt.x-r;
        pt1.y=vCurrentKeys[curr_id].pt.y-r;
        pt2.x=vCurrentKeys[curr_id].pt.x+r;
        pt2.y=vCurrentKeys[curr_id].pt.y+r;

        pt3.x=vLastKeys[last_id].pt.x-r;
        pt3.y=vLastKeys[last_id].pt.y-r;
        pt4.x=vLastKeys[last_id].pt.x+r;
        pt4.y=vLastKeys[last_id].pt.y+r;


        //cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im,vCurrentKeys[curr_id].pt,2,cv::Scalar(0,255,0),-1);

        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(0,255,0));
        cv::circle(last_Im,vLastKeys[last_id].pt,2,cv::Scalar(255,0,0),-1);


    }



    curr_Im.copyTo(imMatch(cv::Rect(0,0,curr_Im.cols,curr_Im.rows)));
    last_Im.copyTo(imMatch(cv::Rect(0,curr_Im.rows,curr_Im.cols,curr_Im.rows)));
    /*now we need to draw the lines*/

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt_curr,pt_last;

        pt_curr = vCurrentKeys[curr_id].pt;
        pt_last.x = vLastKeys[last_id].pt.x;
        pt_last.y = vLastKeys[last_id].pt.y + curr_Im.rows;


        cv::line(imMatch, pt_curr, pt_last, cv::Scalar(0,0,255),1,8);


    }

    //    stringstream ss;
    //    ss << mCurrentFrame.mnId;
    //    string tmp;
    //    ss >> tmp;
    //    string filename = "/home/m/ws_orb2/src/ORB_SLAM2/Matches/" + tmp + ".png";
    //    imwrite(filename, imMatch);
    //    cout << BOLDCYAN"write " << tmp << " suscessfully!" << endl;
    //    cv::resize(imMatch,imMatch,cv::Size(),0.5,0.5);
    cv::imshow("matches",imMatch);
    cv::waitKey(0);


    return imMatch;
}


/*we need a simple function to estimate the inliers and pose*/
RESULT_OF_PNP motionEstimate(Frame &frame1, Frame &frame2)
{
    RESULT_OF_PNP result;
    map<int,int> matches = frame2.matchesId;
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    Camera* pCamera = new Camera();

    for(map<int,int>::const_iterator mit = matches.begin(); mit!=matches.end(); mit++)
    {

        cv::Point2f p = frame1.mvKeys[mit->second].pt;
        pts_img.push_back( cv::Point2f( frame2.mvKeys[mit->first].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point2f pt ( p.x, p.y );
        float depth = frame1.mvDepth[mit->second];

        cv::Point3f pd = pCamera->pixel2camera(pt,depth);
        pts_obj.push_back( pd );
    }


    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }




    double camera_matrix_data[3][3] = {
        {(double)pCamera->fx_, 0, (double)pCamera->cx_},
        {0, (double)pCamera->fy_, (double)pCamera->cy_},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 200, 2.0, 0.99, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    //    cout << "inliers = " <<  endl << inliers << endl;

    /*remove the outliers*/
    map<int,int> matches_out;
    for(int i = 0; i < inliers.rows; i++)
    {
        int row = inliers.at<int>(i);
        map<int,int>::const_iterator map_iter_start = matches.begin();

        if (row == 0)
        {
            matches_out.insert(make_pair(map_iter_start->first,map_iter_start->second));
        }
        else
        {
            while(row)
            {
                map_iter_start++;
                row--;
            }
            matches_out.insert(make_pair(map_iter_start->first,map_iter_start->second));
        }


    }

    cout << "matches out.size = " << matches_out.size() << endl;
    frame2.matchesId.clear();
    frame2.matchesId = matches_out;

    return result;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

