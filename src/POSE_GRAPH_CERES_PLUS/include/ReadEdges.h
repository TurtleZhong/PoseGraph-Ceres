#ifndef READEDGES_H
#define READEDGES_H

#include "common_include.h"

namespace POSE_GRAPH
{

map< int,vector<int> > getEdegsCandidateIndex()
{
    map< int,vector<int> > candidates_index;

    ifstream inFile;
    inFile.open("../config/Edge_Candidates_index.txt"); /*you can Modified it*/

    int i = 1;
    while (inFile.good())
    {
        string s;
        getline(inFile,s);
        //cout << s << endl;

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

    return candidates_index;
}


}



#endif // READEDGES_H
