/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mcurr_Im = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mlast_Im = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

std::vector<Mat> FrameDrawer::DrawFrame()
{
    std::vector<Mat> frameToDraw;
    frameToDraw.reserve(2);
    cv::Mat im,im1;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    std::vector<MapPoint*> vMapPoints;
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    bool reProjectFlag = false;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);
        mIm.copyTo(im1);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vMapPoints = mvpMapPoints;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
    {
        cvtColor(im,im,CV_GRAY2BGR);
        cvtColor(im1,im1,CV_GRAY2BGR);
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
                cv::line(im1,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        const float &fx = mCurrentFrame.fx;
        const float &fy = mCurrentFrame.fy;
        const float &cx = mCurrentFrame.cx;
        const float &cy = mCurrentFrame.cy;
        const int   &minX = mCurrentFrame.mnMinX;
        const int   &maxX = mCurrentFrame.mnMaxX;
        const int   &minY = mCurrentFrame.mnMinY;
        const int   &maxY = mCurrentFrame.mnMaxY;

        //cout << "rec = " << minY << " " << maxY << endl;



        /*The ground truth*/
        if(mCurrentFrame.mnId > 2)
        {
            reProjectFlag = true;
        }
        cv::Mat Rcw =  gd.getFrameRcw(mCurrentFrame.mnId);
        cv::Mat tcw =  gd.getFrametcw(mCurrentFrame.mnId);

        drawOriginfeatures(im1); /*add by zhong*/

        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        //cout << "frameID" << mCurrentFrame.mnId << endl;
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2,pt3,pt4;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    //cout << "pos is :" << endl << ((MapPoint*)(mCurrentFrame.mvpMapPoints[i]))->GetWorldPos() << endl;
                    if(reProjectFlag)
                    {
                        cv::Mat P_w(3,1,CV_64FC1);
                        if(&mCurrentFrame == NULL)
                        {
                            std::vector<cv::Mat> a;
                            return a;
                        }
                        MapPoint* mp = vMapPoints[i];
                        cv:: Mat tmp = mp->GetWorldPos();
                        //                        cv:: Mat tmp = (mCurrentFrame.mvpMapPoints[i])->GetWorldPos();

                        //Mat tmp = Mat::ones(3,1,CV_32FC1);
                        P_w.at<double>(0) = (double)tmp.at<float>(0);
                        P_w.at<double>(1) = (double)tmp.at<float>(1);
                        P_w.at<double>(2) = (double)tmp.at<float>(2);

                        if(P_w.at<double>(2) > 0.0f)
                        {
                            cv::Mat P_c = Rcw * P_w + tcw;
                            Point2f p;
                            p.x = (fx*P_c.at<double>(0))/(P_c.at<double>(2)) + cx ;
                            p.y = (fy*P_c.at<double>(1))/(P_c.at<double>(2)) + cy ;

                            pt3.x=p.x-r;
                            pt3.y=p.y-r;
                            pt4.x=p.x+r;
                            pt4.y=p.y+r;

                            if(p.x > minX && p.x < maxX && p.y > minY && p.y < maxY)
                            {
                                //                                cv::rectangle(im,pt3,pt4,cv::Scalar(255,0,0));
                                //                                cv::circle(im,p,2,cv::Scalar(255,0,0),-1);
                                //                                cv::line(im,p,vCurrentKeys[i].pt,Scalar(0,0,255));
                            }
                        }


                    }


                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);

                    //imwrite(filename,im);


                    mnTracked++;

                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
        stringstream ss;
        ss << mCurrentFrame.mnId;
        string tmp;
        ss >> tmp;
        string filename = "/home/m/ws_orb2/src/ORB_SLAM2/Compare/" + tmp + ".png";
        //        cout << "filename = " << filename << endl;
        //        cout << "frameID = " << mCurrentFrame.mnId << endl;
        //imwrite(filename,im);
        //cout << GREEN"write frame suscess!" << endl;
    }



    cv::Mat imWithInfo,imWithAllfeatures;
    DrawTextInfo(im,state, imWithInfo);
    DrawTextInfo(im1,state, imWithAllfeatures,1);

    frameToDraw.push_back(imWithInfo);
    frameToDraw.push_back(imWithAllfeatures);

    return frameToDraw;
}


cv::Mat FrameDrawer::DrawFrameMatch()
{
    cv::Mat imMatch = Mat::zeros(mIm.rows*2,mIm.cols,CV_8UC3);
    vector<cv::KeyPoint> vCurrentKeys = mCurrentFrame.mvKeys;
    vector<cv::KeyPoint> vLastKeys    = mLastFrame.mvKeys;
    cv::Mat curr_Im, last_Im;


    mcurr_Im.copyTo(curr_Im);
    mlast_Im.copyTo(last_Im);

    if(curr_Im.channels()<3) //this should be always true
    {
        cvtColor(curr_Im,curr_Im,CV_GRAY2BGR);

    }

    if(last_Im.channels()<3) //this should be always true
    {
        cvtColor(last_Im,last_Im,CV_GRAY2BGR);

    }

    /*matches*/
    map<int,int> matches = mMatchsId;
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

    stringstream ss;
    ss << mCurrentFrame.mnId;
    string tmp;
    ss >> tmp;
    string filename = "/home/m/ws_orb2/src/ORB_SLAM2/Matches/" + tmp + ".png";
    imwrite(filename, imMatch);
    cout << BOLDCYAN"write " << tmp << " suscessfully!" << endl;
    cv::resize(imMatch,imMatch,cv::Size(),0.5,0.5);


    return imMatch;
}

vector<Mat> FrameDrawer::DrawReprojectionError()
{

    /*new add write the 3D - 2D pairs*/
    ofstream outFile;




    /*for matches*/
    cv::Mat imMatch = Mat::zeros(mIm.rows*2,mIm.cols,CV_8UC3);


    cv::Mat imReprojection = Mat::zeros(mIm.rows*2,mIm.cols,CV_8UC3);
    vector<cv::KeyPoint> vCurrentKeys = mCurrentFrame.mvKeys;
    vector<cv::KeyPoint> vLastKeys    = mLastFrame.mvKeys;
    vector<float> vDepth = mLastFrame.mvDepth; /*depth of last frame*/
    int lastFrameId = mLastFrame.mnId;
    int currFrameId = mCurrentFrame.mnId;

    stringstream ss3;
    ss3 << currFrameId;
    string tmp3;
    ss3 >> tmp3;
    string pairsName = "/home/m/ws_orb2/src/ORB_SLAM2/3d-2dpairs/" + tmp3+ ".txt";
    outFile.open(pairsName);





    cv::Mat Tlw = mLastFrame.mTcw;
    cv::Mat Tcw = mCurrentFrame.mTcw;

    cv::Mat Tcl = Tcw * Tlw.inv();
    cv::Mat Rcl = Tcl.rowRange(0,3).colRange(0,3);
    cv::Mat tcl = Tcl.rowRange(0,3).col(3);
    //cv::normalize(tcl,tcl);

    /*now we need to load the grodund truth poses to reproject the point*/
    cv::Mat Twl_gd = gd.getFrameTwc(lastFrameId);
    cv::Mat Tcw_gd = gd.getFrameTwc(currFrameId).inv();
    cv::Mat Tcl_gd = Tcw_gd * Twl_gd;
    cv::Mat Rcl_gd = Tcl_gd.rowRange(0,3).colRange(0,3);
    cv::Mat tcl_gd = Tcl_gd.rowRange(0,3).col(3);


    Rcl_gd.convertTo(Rcl_gd,CV_32FC1);
    tcl_gd.convertTo(tcl_gd,CV_32FC1);
    //cv::normalize(tcl_gd,tcl_gd);

    cout << endl << BOLDMAGENTA"Tcl_gd = " << endl << Tcl_gd << endl;


    cv::Mat curr_Im, last_Im, curr_Im_match, last_Im_match;



    mcurr_Im.copyTo(curr_Im);
    mlast_Im.copyTo(last_Im);

    if(curr_Im.channels()<3) //this should be always true
    {
        cvtColor(curr_Im,curr_Im,CV_GRAY2BGR);

    }

    if(last_Im.channels()<3) //this should be always true
    {
        cvtColor(last_Im,last_Im,CV_GRAY2BGR);

    }

    curr_Im.copyTo(curr_Im_match);
    last_Im.copyTo(last_Im_match);

    /*matches*/
    map<int,int> matches = mMatchsId;
    const float r = 5;

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /* since we got the id, we can get the mappoints from last image
         * and we need to reproject it
         */

        /*part2: draw the match*/
        //cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im_match,vCurrentKeys[curr_id].pt,1,cv::Scalar(0,255,0),-1);

        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(0,255,0));
        cv::circle(last_Im_match,vLastKeys[last_id].pt,1,cv::Scalar(255,0,0),-1);

        int      u = vLastKeys[last_id].pt.x;
        int      v = vLastKeys[last_id].pt.y;


        float depth = vDepth[last_id];
        //float depth = 1.0;

        cv::Mat x3Dl = pixel2Camera(u, v, depth);

        /*ground truth*/
        cv::Mat x3Dl_gd = pixel2Camera(u,v,depth);



        /*now we got the 3D point in last frame*/
        /*next we need to get the Tcl. c-->current frame   l--> lastframe*/

        /*before we show it we need to check weather the x3Dl is empty*/
        if(!x3Dl.empty() && !x3Dl_gd.empty())
        {
            outFile << x3Dl.at<float>(0) << " " << x3Dl.at<float>(1) << " " << x3Dl.at<float>(2) << " ";
            outFile << vCurrentKeys[curr_id].pt.x << " " << vCurrentKeys[curr_id].pt.y << endl;

            cv::Mat x3Dc = Rcl * x3Dl + tcl;
            cv::Mat x3Dc_gd = Rcl_gd * x3Dl_gd + tcl_gd;

            /*OK, let's reproject this point*/

            Point2f pixel = camera2Pixel(x3Dc);
            Point2f pixel_gd = camera2Pixel(x3Dc_gd);

            /*now we have reproject the point which in last frame to the current frame*/

            /*let's show it and draw a line between the current match features*/

            cv::circle(curr_Im,pixel,1,cv::Scalar(255,0,0),-1);
            cv::circle(curr_Im,pixel_gd,1,cv::Scalar(0,0,255),-1);

            cv::line(curr_Im,vCurrentKeys[curr_id].pt, pixel, Scalar(255,0,0),1);
            cv::line(curr_Im,vCurrentKeys[curr_id].pt, pixel_gd, Scalar(0,0,255),1);
        }



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
        cv::circle(curr_Im,vCurrentKeys[curr_id].pt,1,cv::Scalar(0,255,0),-1);


        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(255,0,0));
        cv::circle(last_Im,vLastKeys[last_id].pt,1,cv::Scalar(255,0,0),-1);

        /*show the depth*/
        stringstream s;
        s << depth;
        cout << "*****" << "vLastKeys[last_id].pt.x = " << vLastKeys[last_id].pt.x << " vLastKeys[last_id].pt.y = " << vLastKeys[last_id].pt.y << endl;
        cv::putText(last_Im, s.str(), vLastKeys[last_id].pt, cv::FONT_HERSHEY_PLAIN,0.5,cv::Scalar(255,255,255),1,8 );


    }

    curr_Im_match.copyTo(imMatch(cv::Rect(0,0,curr_Im_match.cols,curr_Im_match.rows)));
    last_Im_match.copyTo(imMatch(cv::Rect(0,curr_Im_match.rows,curr_Im_match.cols,curr_Im_match.rows)));
    /*prat2:now we need to draw the lines*/
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




    curr_Im.copyTo(imReprojection(cv::Rect(0,0,curr_Im.cols,curr_Im.rows)));
    last_Im.copyTo(imReprojection(cv::Rect(0,curr_Im.rows,curr_Im.cols,curr_Im.rows)));


    stringstream ss1;
    ss1 << currFrameId;
    string tmp;
    ss1 >> tmp;
    string matchsName = "/home/m/ws_orb2/src/ORB_SLAM2/Matches/" + tmp + ".png";
    //imwrite(matchsName, imMatch);
    cout << BOLDCYAN"write " << tmp << " suscessfully!" << endl;

    //resize(imReprojection,imReprojection,Size(),0.7,0.8);
//    stringstream ss;
//    ss << currFrameId;

    string filename = "/home/m/ws_orb2/src/ORB_SLAM2/Reprojection/" + tmp + ".png";
    //imwrite(filename,imReprojection);

    vector<Mat> output;
    output.push_back(imReprojection);
    output.push_back(imMatch);









    return output;


}

/*add by zhong*/
void FrameDrawer::drawOriginfeatures(cv::Mat &im1)
{
    vector<cv::KeyPoint> vCurrentKeys;
    vCurrentKeys = mvCurrentKeys;
    const float r = 5;
    const int n = vCurrentKeys.size();
    for(int i=0;i<n;i++)
    {
        cv::Point2f pt1,pt2;
        pt1.x=vCurrentKeys[i].pt.x-r;
        pt1.y=vCurrentKeys[i].pt.y-r;
        pt2.x=vCurrentKeys[i].pt.x+r;
        pt2.y=vCurrentKeys[i].pt.y+r;

        // This is a match to a MapPoint in the map
        cv::rectangle(im1,pt1,pt2,cv::Scalar(0,0,255));
        cv::circle(im1,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
    }
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText, int flag)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        if(flag == 0)
        {
            s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked << " | ID: " << mCurrentFrame.mnId;
        }
        else if (flag == 1)
        {
            s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Features: " << mvCurrentKeys.size() << " | ID: " << mCurrentFrame.mnId;
        }

        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mCurrentFrame = pTracker->mCurrentFrame;
    //cout << "Current frame id is: " << mCurrentFrame.mnId << endl;

    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    //cout << mCurrentFrame.mvpMapPoints.size() << endl; //test the mppoint size

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        mvpMapPoints = pTracker->mCurrentFrame.mvpMapPoints;
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

void FrameDrawer::UpdateFrame2Frame(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);

    //pTracker->mImGray.copyTo(mlast_Im);
    pTracker->getLastFrame().mImageGray.copyTo(mlast_Im);
    pTracker->mImGray.copyTo(mcurr_Im);

    mLastFrame = pTracker->getLastFrame();
    mCurrentFrame = pTracker->mCurrentFrame;
    if(mCurrentFrame.mnId > 5)
        cout << RED << pTracker->mCurrentFrame.matchesId.size() << endl;
    //cout << "Current frame id is: " << mCurrentFrame.mnId << endl;
    mMatchsId = pTracker->mCurrentFrame.matchesId;

    mState=static_cast<int>(pTracker->mLastProcessedState);
}

cv::Mat FrameDrawer::pixel2Camera(const int u, const int v, const float depth)
{
    const float z = depth;
    const float cx = mCurrentFrame.cx;
    const float cy = mCurrentFrame.cy;
    const float invfx = mCurrentFrame.invfx;
    const float invfy = mCurrentFrame.invfy;
    if(z>0)
    {
        const double x = (u-cx)*z*invfx;
        const double y = (v-cy)*z*invfy;

        cv::Mat x3D = (cv::Mat_<float>(3,1) << x, y, z);
        return x3D;
    }
    else
        return cv::Mat();
}

cv::Point2f FrameDrawer::camera2Pixel(Mat &x3D)
{
    float x = x3D.at<float>(0,0);
    float y = x3D.at<float>(1,0);
    float z = x3D.at<float>(2,0);

    const float cx = mCurrentFrame.cx;
    const float cy = mCurrentFrame.cy;
    const float fx = mCurrentFrame.fx;
    const float fy = mCurrentFrame.fy;

    if(z>0)
    {
        Point2f pixel;
        pixel.x = fx * (x/z) + cx;
        pixel.y = fy * (y/z) + cy;
        return pixel;
    }
    else
        return Point2f();

}





} //namespace ORB_SLAM
