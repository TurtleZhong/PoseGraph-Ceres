/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include "viso_stereo.h"
#include <png++/png.hpp>
#include <opencv2/opencv.hpp>
#include <map>


using namespace std;
using namespace cv;

Mat drawFeatureMatches(Mat imLeft, Mat imRight, std::vector<Matcher::p_match> featureMatches, int flag);
Mat drawCircularMatches(Mat currImLeft, Mat currImRight, Mat lastImLeft, Mat lastImRight, std::vector<Matcher::p_match> featureMatches);

int main (int argc, char** argv) {

    // we need the path name to 2010_03_09_drive_0019 as input argument
    if (argc<2) {
        cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
        return 1;
    }

    // sequence directory
    string dir = argv[1];

    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    VisualOdometryStereo::parameters param;

    // calibration parameters for sequence 2010_03_09_drive_0019
    param.calib.f  = 645.24; // focal length in pixels
    param.calib.cu = 635.96; // principal point (u-coordinate) in pixels
    param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
    param.base     = 0.5707; // baseline in meters

    // init visual odometry
    VisualOdometryStereo viso(param);

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    Matrix pose = Matrix::eye(4);

    Mat previous_imLeft,previous_imRight;
    
    // loop through all frames i=0:372
    for (int32_t i=0; i<1000; i++) {

        // input file names
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir + "/I1_" + base_name;
        string right_img_file_name = dir + "/I2_" + base_name;

        // catch image read/write errors here
        try {

            // load left and right input image
            png::image< png::gray_pixel > left_img(left_img_file_name);
            png::image< png::gray_pixel > right_img(right_img_file_name);




            // image dimensions
            int32_t width  = left_img.get_width();
            int32_t height = left_img.get_height();

            // convert input images to uint8_t buffer
            uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            int32_t k=0;
            for (int32_t v=0; v<height; v++) {
                for (int32_t u=0; u<width; u++) {
                    left_img_data[k]  = left_img.get_pixel(u,v);
                    right_img_data[k] = right_img.get_pixel(u,v);
                    k++;
                }
            }

            // status
            cout << "Processing: Frame: " << i;

            Mat imLeft(height,width,CV_8UC1,left_img_data);
            Mat imRight(imLeft.size(), CV_8UC1, right_img_data);



            // compute visual odometry
            int32_t dims[] = {width,height,width};
            if (viso.process(left_img_data,right_img_data,dims)) {

                /*plot the right matchs add by zhong*/
                if(i > 3)
                {
                    std::vector<Matcher::p_match> featureMatchs = viso.getMatches();
//                    Mat currentFrame = drawFeatureMatches(imLeft,imRight,featureMatchs,0);
//                    Mat previousFrame = drawFeatureMatches(previous_imLeft,previous_imLeft,featureMatchs,1);
//                    imshow("currentFrame",currentFrame);
//                    imshow("previousFrame",previousFrame);

                    Mat output = drawCircularMatches(imLeft,imRight,previous_imLeft,previous_imRight,featureMatchs);
                    imshow("output",output);
                }



                // on success, update current pose
                pose = pose * Matrix::inv(viso.getMotion());

                // output some statistics
                double num_matches = viso.getNumberOfMatches();
                double num_inliers = viso.getNumberOfInliers();
                cout << ", Matches: " << num_matches;
                cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
                cout << pose << endl << endl;

            } else {
                cout << " ... failed!" << endl;
            }

            cv::waitKey(27);

            previous_imLeft = imLeft;
            previous_imRight = imRight;
            // release uint8_t buffers
            free(left_img_data);
            free(right_img_data);

            // catch image read errors here
        } catch (...) {
            cerr << "ERROR: Couldn't read input files!" << endl;
            return 1;
        }
    }

    // output
    cout << "Demo complete! Exiting ..." << endl;

    // exit
    return 0;
}


Mat drawFeatureMatches(Mat imLeft, Mat imRight, std::vector<Matcher::p_match> featureMatches, int flag = 0)
{
    /*
     * flag = 0 : draw current frame's feature;
     * flag = 1 : draw previous frame's feature;
     */

    /*
     * 1:left image
     * 2:right image
     */
    //const float r = 5;


    cv::cvtColor(imLeft,imLeft,CV_GRAY2BGR);
    cv::cvtColor(imRight,imRight,CV_GRAY2BGR);
    Mat output = Mat::zeros(imLeft.rows, imLeft.cols * 2, imLeft.type());
    vector < vector<Point2i> > position;

    if (flag == 0)
    {
        for (int i = 0; i < featureMatches.size(); i++)
        {
            const int &u1c = Matcher::p_match(featureMatches[i]).u1c;
            const int &v1c = Matcher::p_match(featureMatches[i]).v1c;
            const int &u2c = Matcher::p_match(featureMatches[i]).u2c;
            const int &v2c = Matcher::p_match(featureMatches[i]).v2c;

            Point2i pt1_center,pt2_center;
            pt1_center.x = u1c;
            pt1_center.y = v1c;
            pt2_center.x = u2c;
            pt2_center.y = v2c;

            vector<Point2i> tmp;

            tmp.push_back(pt1_center);
            tmp.push_back(pt2_center);

            position.push_back(tmp);


            cv::circle(imLeft,pt1_center,2,Scalar(0,255,0),-1);
            cv::circle(imRight,pt2_center,2,Scalar(0,255,0),-1);

        }


        imLeft.copyTo(output(cv::Rect(0,0,imLeft.cols,imLeft.rows)));
        imRight.copyTo(output(cv::Rect(imRight.cols,0,imRight.cols,imRight.rows)));

        for(vector < vector<Point2i> >::iterator iter = position.begin(); iter!=position.end();iter++)
        {
            vector<Point2i> pos = *iter;
            Point2i pt_left = pos[0];
            Point2i pt_right = pos[1];

            cv::line(output,pt_left,Point2i(pt_right.x + imLeft.cols, pt_right.y),Scalar(0,0,255),1,8);
        }




        cv::resize(output,output,Size(),0.7,0.8);
    }
    else
    {
        for (int i = 0; i < featureMatches.size(); i++)
        {
            const int &u1p = Matcher::p_match(featureMatches[i]).u1p;
            const int &v1p = Matcher::p_match(featureMatches[i]).v1p;
            const int &u2p = Matcher::p_match(featureMatches[i]).u2p;
            const int &v2p = Matcher::p_match(featureMatches[i]).v2p;

            Point2i pt1_center,pt2_center;
            pt1_center.x = u1p;
            pt1_center.y = v1p;
            pt2_center.x = u2p;
            pt2_center.y = v2p;

            vector<Point2i> tmp;

            tmp.push_back(pt1_center);
            tmp.push_back(pt2_center);

            position.push_back(tmp);


            cv::circle(imLeft,pt1_center,2,Scalar(0,255,0),-1);
            cv::circle(imRight,pt2_center,2,Scalar(0,255,0),-1);

        }


        imLeft.copyTo(output(cv::Rect(0,0,imLeft.cols,imLeft.rows)));
        imRight.copyTo(output(cv::Rect(imRight.cols,0,imRight.cols,imRight.rows)));

        for(vector < vector<Point2i> >::iterator iter = position.begin(); iter!=position.end();iter++)
        {
            vector<Point2i> pos = *iter;
            Point2i pt_left = pos[0];
            Point2i pt_right = pos[1];

            cv::line(output,pt_left,Point2i(pt_right.x + imLeft.cols, pt_right.y),Scalar(0,0,255),1,8);
        }




        cv::resize(output,output,Size(),0.7,0.8);
    }


    return output;

}

Mat drawCircularMatches(Mat currImLeft, Mat currImRight, Mat lastImLeft, Mat lastImRight, std::vector<Matcher::p_match> featureMatches)
{

    cv::cvtColor(currImLeft,currImLeft,CV_GRAY2BGR);
    cv::cvtColor(currImRight,currImRight,CV_GRAY2BGR);
    cv::cvtColor(lastImLeft,lastImLeft,CV_GRAY2BGR);
    cv::cvtColor(lastImRight,lastImRight,CV_GRAY2BGR);
    Mat output = Mat::zeros(currImLeft.rows * 2, currImLeft.cols * 2, currImLeft.type());
    vector < vector<Point2i> > position;

    int cols = currImLeft.cols;
    int rows = currImLeft.rows;

    currImLeft.copyTo(output(cv::Rect(0,0,cols,rows)));
    currImRight.copyTo(output(cv::Rect(cols,0,cols,rows)));
    lastImLeft.copyTo(output(cv::Rect(0,rows,cols,rows)));
    lastImRight.copyTo(output(cv::Rect(cols,rows,cols,rows)));

    for(int i = 0; i < featureMatches.size(); i++)
    {
        const int &u1c = Matcher::p_match(featureMatches[i]).u1c;
        const int &v1c = Matcher::p_match(featureMatches[i]).v1c;
        const int &u2c = Matcher::p_match(featureMatches[i]).u2c + cols;
        const int &v2c = Matcher::p_match(featureMatches[i]).v2c;
        const int &u1p = Matcher::p_match(featureMatches[i]).u1p;
        const int &v1p = Matcher::p_match(featureMatches[i]).v1p + rows;
        const int &u2p = Matcher::p_match(featureMatches[i]).u2p + cols;
        const int &v2p = Matcher::p_match(featureMatches[i]).v2p + rows;

        Point2i pt1_center,pt2_center,pt3_center,pt4_center;
        pt1_center.x = u1c;
        pt1_center.y = v1c;
        pt2_center.x = u2c;
        pt2_center.y = v2c;
        pt3_center.x = u1p;
        pt3_center.y = v1p;
        pt4_center.x = u2p;
        pt4_center.y = v2p;

        cv::circle(output,pt1_center,2,Scalar(0,255,0),-1);
        cv::circle(output,pt2_center,2,Scalar(0,255,0),-1);
        cv::circle(output,pt3_center,2,Scalar(0,255,0),-1);
        cv::circle(output,pt4_center,2,Scalar(0,255,0),-1);


        cv::line(output,pt1_center,pt3_center,Scalar(255,0,0),1,8);
        cv::line(output,pt3_center,pt4_center,Scalar(0,255,0),1,8);
        cv::line(output,pt4_center,pt2_center,Scalar(0,0,255),1,8);
        cv::line(output,pt2_center,pt1_center,Scalar(255,0,0),1,8);


    }



    resize(output,output,Size(),0.7,0.8);
    return output;



}








