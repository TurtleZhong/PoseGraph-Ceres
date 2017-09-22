#include "ImageRead.h"

using namespace POSE_GRAPH;


ImageReader::ImageReader()
{
    string timestamp_path = Config::get<string>("timestamp_path");
    string sequence_dir = Config::get<string>("sequence_dir");
    ifstream inFile;
    inFile.open(timestamp_path);
    while (inFile.good())
    {
        string timestamp;
        inFile >> timestamp;
        timestamp += ".png";
        string leftImagePath = sequence_dir + "/image_0/" + timestamp;
        string rightImagePath = sequence_dir + "/image_1/" +timestamp;
        vector<string> path;
        path.push_back(leftImagePath);
        path.push_back(rightImagePath);

        mvImagePath.push_back(path);
    }

}

ImageReader::~ImageReader()
{

}

vector<string> ImageReader::getImagePath(const int &frameId)
{
    vector<string> path = mvImagePath[frameId];

    return path;
}
