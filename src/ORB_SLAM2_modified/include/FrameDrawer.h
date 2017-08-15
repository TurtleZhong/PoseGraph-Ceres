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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include "GroundTruth.h"
#include "Frame.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<sstream>

#include<mutex>



namespace ORB_SLAM2
{

class Tracking;
class Viewer;
class Frame;
class GroundTruth;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    /*add by zhong 用于显示两帧之间的feature对应*/
    void UpdateFrame2Frame(Tracking *pTracker);
    cv::Mat DrawFrameMatch();


    // Draw last processed frame.
    /*edit it cuz we need to test some functions origin:cv::Mat DrawFrame();*/

    std::vector<cv::Mat> DrawFrame();
    /*add by zhong: some test functions*/
    void drawOriginfeatures(cv::Mat &im1); /*because some features are recognized outliers*/
    /*add by zhong*/
    void DrawBox(cv::Mat &im,vector<cv::Mat> &matchsId );
    Frame mCurrentFrame;
    GroundTruth gd;
    std::vector <MapPoint*> mvpMapPoints;

    Frame mLastFrame;
    map<int,int> mMatchsId;

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText, int flag = 0);

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    /*add by zhong----frame2frame show*/
    cv::Mat mlast_Im;
    cv::Mat mcurr_Im;




    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
