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


        cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im,vCurrentKeys[curr_id].pt,2,cv::Scalar(255,0,0),-1);

        cv::rectangle(last_Im,pt3,pt4,cv::Scalar(0,255,0));
        cv::circle(last_Im,vLastKeys[last_id].pt,2,cv::Scalar(0,255,0),-1);


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
    cv::resize(imMatch,imMatch,cv::Size(),0.5,0.5);
    stringstream ss;
    ss << mCurrentFrame.mnId;
    string tmp;
    ss >> tmp;
    string filename = "/home/m/ws_orb2/src/ORB_SLAM2/Matches/" + tmp + ".png";
    imwrite(filename, imMatch);
    cout << "write " << tmp << " suscessfully!" << endl;





    return imMatch;
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






} //namespace ORB_SLAM
