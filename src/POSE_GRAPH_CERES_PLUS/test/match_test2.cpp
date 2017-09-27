#include "common_include.h"
#include "config.h"
#include "Frame.h"
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
    if(argc!=3)
    {
        cout << "Useage: ./match_test2 currentFrame.id lastFrame.id" << endl;
        return 1;
    }


    string argv1 = argv[1];
    string argv2 = argv[2];
    int currentFrameIndex,lastFrameIndex;
    stringstream ss1;
    ss1 << argv1;
    ss1 >> currentFrameIndex;
    stringstream ss2;
    ss2 << argv2;
    ss2 >> lastFrameIndex;

    cout << "id " << currentFrameIndex << " " << lastFrameIndex << endl;





    Config::setParameterFile("../config/config00.yaml");
    string dir = Config::get<string>("sequence_dir");

    GroundTruth gd;

    SequenceRun* sequenceRun = new SequenceRun();
    Frame currentFrame, lastFrame;


    char base_name[256];
    sprintf(base_name,"%06d.png",currentFrameIndex);
    string left_img_file_name = dir + "/image_0/" + base_name;
    string right_img_file_name = dir + "/image_1/" + base_name;
    Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
    Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);

    char base_name1[256];
    sprintf(base_name1,"%06d.png",lastFrameIndex);
    string left_img_file_name1 = dir + "/image_0/" + base_name1;
    string right_img_file_name1 = dir + "/image_1/" + base_name1;
    Mat Left1 = cv::imread(left_img_file_name1,CV_LOAD_IMAGE_GRAYSCALE);
    Mat Right1 = cv::imread(right_img_file_name1,CV_LOAD_IMAGE_GRAYSCALE);


    sequenceRun->GrabImageStereo(Left,Right,0.0);
    currentFrame = sequenceRun->mCurrentFrame;

    sequenceRun->GrabImageStereo(Left1,Right1,0.0);
    lastFrame = sequenceRun->mCurrentFrame;





    /*Do the feature matching test*/

    ORBmatcher matcher(0.7, false);


    int nmatches = matcher.MatcheTwoFrames(currentFrame,lastFrame,false);
    cout << "we got " << nmatches << " matches!" << endl;

    RESULT_OF_PNP result;

    result = motionEstimate(lastFrame,currentFrame);
    DrawFrameMatch(currentFrame,lastFrame);

    cout << "tvec = \n" << result.tvec << endl;
    cout << "inliers = " << result.inliers << endl;

    /*groundTruth*/
    cv::Mat Tcw = gd.getFrameTcw(currentFrameIndex);
    cv::Mat Twc = gd.getFrameTwc(lastFrameIndex);

    cv::Mat Tcl = Tcw * Twc;
    cv::Mat tcl = Tcl.rowRange(0,3).col(3);
    cout << "GroundTruth tcl = " << tcl << endl;




    cv::imshow("currentFrame", currentFrame.mImageGray);


    cv::waitKey(0);






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
    double fx_ = Config::get<double>("Camera.fx");
    double fy_ = Config::get<double>("Camera.fy");
    double cx_ = Config::get<double>("Camera.cx");
    double cy_ = Config::get<double>("Camera.cy");

    RESULT_OF_PNP result;
    map<int,int> matches = frame2.matchesId;
    vector<cv::Point3f> pts_obj;
    vector< cv::Point2f > pts_img;


    for(map<int,int>::const_iterator mit = matches.begin(); mit!=matches.end(); mit++)
    {

        cv::Point2f p = frame1.mvKeys[mit->second].pt;
        pts_img.push_back( cv::Point2f( frame2.mvKeys[mit->first].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point2f pt ( p.x, p.y );
        float depth = frame1.mvDepth[mit->second];

        //cv::Point3f pd = pCamera->pixel2camera(pt,depth);
        cv::Mat p3d = frame2.pixel2Camera(pt.x,pt.y,depth);
        cv::Point3f pd(p3d.at<float>(0),p3d.at<float>(1),p3d.at<float>(2));
        pts_obj.push_back( pd );
    }


    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }


    double camera_matrix_data[3][3] = {
        {fx_, 0, cx_},
        {0, fy_, cy_},
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

    //cout << "matches out.size = " << matches_out.size() << endl;
    frame2.matchesId.clear();
    frame2.matchesId = matches_out;

    return result;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

