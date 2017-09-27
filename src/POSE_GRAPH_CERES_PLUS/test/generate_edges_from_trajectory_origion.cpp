#include "common_include.h"
#include "GroundTruth.h"
#include "config.h"
#include <fstream>
#include <sstream>
using namespace POSE_GRAPH;
using namespace std;


/*remember that trajectory origin format
 * x y z q_x q_y q_z q_w
 * we just need to use the x y z three cols
 * to get the edges_candidates_index
 */

/*Edge_Candidates_index.txt Format
 *        0              1                2...
 * currentFrame.id nearbyFrame.id (search_range_candidate.frame.id)
 */

ofstream outFile;
vector<int> getCandidatesIndex(const int &curr_id,vector<Mat> &poses);
bool isInSearchRange(const int curr_id, const int last_id, vector<Mat> &poses);

int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");
    outFile.open("../config/Edge_Candidates_index.txt");

    int sequence_length = Config::get<int>("sequence_length");
    GroundTruth gd;

    vector<Mat> poses = gd.getPoses();


    int id;
    for(id = 1; id < sequence_length; id++)
    {
        cout << BOLDGREEN << "Process: " << id << endl;
        vector<int> vIndex = getCandidatesIndex(id,poses);

        cout << id << ": ";
        outFile << id << " ";

        for(int i = 0; i < vIndex.size(); i++)
        {
             cout << vIndex[i] << " ";
             outFile << vIndex[i] << " ";
        }

        cout << endl;
        outFile << endl;
    }

    return 0;
}

vector<int> getCandidatesIndex(const int &curr_id, vector<Mat> &poses)
{
    vector<int> vIndex;
    vIndex.push_back(curr_id - 1 );

    if(curr_id > 100 )
    {
        for(int i = 0; i < curr_id - 100; i++)
        {
            bool is_candidate = isInSearchRange(curr_id,i,poses);
            if(is_candidate)
            {
                if(curr_id - i > 100)
                {
                    vIndex.push_back(i);
                    cout << BOLDCYAN << "We got a loop" << endl;
                }

            }
        }
    }

    return vIndex;
}


bool isInSearchRange(const int curr_id, const int last_id, vector<Mat> &poses)
{


    cv::Mat twc = Mat(poses[curr_id]).rowRange(0,3).col(3);
    cv::Mat M = Mat(poses[last_id]).rowRange(0,3).col(3);
    float centerA = twc.at<float>(0);
    float centerB = twc.at<float>(1);
    float centerC = twc.at<float>(2);

    float x = M.at<float>(0);
    float y = M.at<float>(1);
    float z = M.at<float>(2);

    float distA = x - centerA;
    float distB = y - centerB;
    float distC = z - centerC;
    float dist = distA*distA + distB*distB + distC*distC;
    //cout << "Dist = " << dist << endl;

    float searchRadius = (float)Config::get<int>("search_radius");

    if(dist > (searchRadius*searchRadius))
        return false;
    else
        return true;

}

