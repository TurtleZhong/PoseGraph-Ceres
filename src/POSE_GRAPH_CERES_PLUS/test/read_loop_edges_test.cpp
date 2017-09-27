#include "common_include.h"
#include "config.h"
#include "Frame.h"
#include "MapPoint.h"
#include "converter.h"
#include "SequenceRun.h"
#include "ORBmatcher.h"
#include "MotionEstimate.h"
#include "GroundTruth.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include "ReadEdges.h"

using namespace POSE_GRAPH;


int main(int argc, char *argv[])
{
    /*loop edge foamat*/
    /*      0                  1               2...*/
    /*currentFrame.id   lastFrame.id loopFrame candidates.id*/
    ifstream inFile;
    inFile.open("../config/Edge_Candidates_index.txt");

    map< int,vector<int> > candidates_index;
    vector<int> firstFrame;
    firstFrame.push_back(0);
    candidates_index[0] = firstFrame;


    int i = 1;
    while (inFile.good())
    {
        string s;
        getline(inFile,s);
        cout << s << endl;

        int index;
        vector<int> vIndex;


        stringstream ss;
        ss << s;

        int j = 0;
        while (ss >> index)
        {
            if(j == 0)
                j = 1;
            else
                vIndex.push_back(index);
            j++;

        }

        candidates_index[i] = vIndex;

        i++;
    }


//    for(map< int,vector<int> >::const_iterator iter = candidates_index.begin();iter!=candidates_index.end();iter++)
//    {
//        cout << iter->first << " ";
//        vector<int> index = iter->second;
//        for(int i = 0; i < index.size(); i++)
//        {
//            cout << index[i] << " ";
//        }
//        cout << endl;
//    }

    getEdegsCandidateIndex();





    return 0;
}
