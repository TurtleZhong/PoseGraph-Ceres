#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"
#include "ORBmatcher.h"

#include <fstream>

using namespace std;
using namespace POSE_GRAPH;

int find_feature_matches(Frame &currentFrame,
                         Frame &lastFrame,
                         vector<DMatch>& matches,
                         string method = "BF");

int main(int argc, char *argv[])
{

    Config::setParameterFile("/home/m/ws_orb2/src/POSE_GRAPH/config/config05.yaml");
    string dir = Config::get<string>("sequence_dir");

    SequenceRun* sequenceRun = new SequenceRun();
    Frame currentFrame, lastFrame;


    char base_name[256];
    sprintf(base_name,"%06d.png",2511);
    string left_img_file_name = dir + "/image_0/" + base_name;
    string right_img_file_name = dir + "/image_1/" + base_name;
    Mat Left = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
    Mat Right = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);

    char base_name1[256];
    sprintf(base_name1,"%06d.png",125);
    string left_img_file_name1 = dir + "/image_0/" + base_name1;
    string right_img_file_name1 = dir + "/image_1/" + base_name1;
    Mat Left1 = cv::imread(left_img_file_name1,CV_LOAD_IMAGE_GRAYSCALE);
    Mat Right1 = cv::imread(right_img_file_name1,CV_LOAD_IMAGE_GRAYSCALE);


    sequenceRun->GrabImageStereo(Left,Right,0.0);
    currentFrame = sequenceRun->mCurrentFrame;

    sequenceRun->GrabImageStereo(Left1,Right1,0.0);
    lastFrame = sequenceRun->mCurrentFrame;






    vector<DMatch> matches;


    //            ORBmatcher matcher(0.7,false);
    //            int nmatches = matcher.MatcheTwoFrames(currentFrame,lastFrame,4,false);
    //            cout << "nmatches = " << nmatches << endl;
    find_feature_matches(currentFrame,lastFrame,matches,"BF");




    cv::imshow("currentFrame", currentFrame.mImageGray);


    cv::waitKey(0);


    return 0;
}



int find_feature_matches(Frame &currentFrame,
                         Frame &lastFrame,
                         vector<DMatch>& matches,
                         string method)
{
    /*
      * define the detect param: use ORB
      */
    Ptr<ORB> orb = ORB::create(1000);
    Mat descriptors;
    std::vector<KeyPoint> keypoints1,keypoints2;
    /*
      * use detect() function to detect keypoints
      */
    orb-> detect(currentFrame.mImageGray, keypoints1);
    /*
      * conpute the extractor and show the keypoints
      */
    orb-> compute(currentFrame.mImageGray, keypoints1, descriptors);

    Mat testDescriptors;
    orb->detect(lastFrame.mImageGray, keypoints2);
    orb->compute(lastFrame.mImageGray, keypoints2,testDescriptors);

    /*
      * FLANN
      */

    if(method == "KNN" || method == "knn")
    {
        flann::Index flannIndex(testDescriptors, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);



        /*Match the feature*/
        Mat matchIndex(descriptors.rows, 2, CV_32S);
        Mat matchDistance(descriptors.rows, 2, CV_32S);
        flannIndex.knnSearch(descriptors.rows, matchIndex, matchDistance, 2, flann::SearchParams());

        //vector<DMatch> goodMatches;
        for (int i = 0; i < matchDistance.rows; i++)
        {
            if(matchDistance.at<float>(i,0) < 0.6 * matchDistance.at<float>(i, 1))
            {
                DMatch dmatchs(i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,1));
                matches.push_back(dmatchs);
            }
        }

    }
    else if(method == "BF")
    {
        vector<DMatch> tmpMatches;
        BFMatcher matcher(NORM_HAMMING);
        matcher.match(descriptors, testDescriptors, tmpMatches);
        double min_dist = 10000, max_dist = 0;
        for(int i = 0; i < descriptors.rows; i++)
        {
            double dist = tmpMatches[i].distance;
            if(dist < min_dist) min_dist = dist;
            if(dist > max_dist) max_dist = dist;
        }
        cout << "--Max dist = " << max_dist << endl;
        cout << "--Min dist = " << min_dist << endl;

        for(int i =0; i < descriptors.rows; i++)
        {
            if(tmpMatches[i].distance <= max(2*min_dist, 35.0))
            {
                matches.push_back(tmpMatches[i]);
            }
        }

    }
    else
    {

    }

    Mat resultImage;
    drawMatches(currentFrame.mImageGray, keypoints1, lastFrame.mImageGray, keypoints2, matches, resultImage);
    imshow("result of Image", resultImage);

    cout << "We got " << matches.size() << " good Matchs" << endl;
    return (int)matches.size();
}
