#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H

#include "common_include.h"
#include "config.h"


using namespace cv;
using namespace std;

namespace POSE_GRAPH
{
class ImageReader
{
public:
    ImageReader();
    ~ImageReader();

    vector<string> getImagePath(const int &frameId);

private:
    vector< vector<string> > mvImagePath;
};

}


#endif // POSE_GRAPH_H
