#include "common_include.h"
#include "config.h"
#include "viso_stereo.h"
#include "camera.h"
#include "frame.h"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
//    if(argc<2)
//    {
//        cerr << "Usage: ./visotest1 path/of/sequence/dir" << endl;
//        return 1;
//    }
    //string dir = argv[1];

    string dir = "/home/m/KITTI/dataset/sequences/00";



    Config::setParameterFile("/home/m/ws_orb2/src/VISO2+/config/KITTI00-02.yaml");
    Camera::Ptr camera (new Camera);

    VisualOdometryStereo::parameters param;
    param.calib.f = camera->fx_;
    param.calib.cu = camera->cx_;
    param.calib.cv = camera->cy_;
    param.base = camera->b_;


    VisualOdometryStereo viso(param);
    Matrix pose = Matrix::eye(4);
    cout << dir << endl << param.calib.f << endl;
    waitKey(0);

    for(int32_t i = 0; i < 4540; i++)
    {
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        Mat Left = imread(left_img_file_name);
        Mat Right = imread(right_img_file_name);
        Frame currentFrame(i,camera,Left,Right);

        if(viso.process(currentFrame.img_left_data,currentFrame.img_right_data,currentFrame.dims))
        {
            imshow("currentFrame",currentFrame.imLeft_);
            std::vector<Matcher::p_match> featureMatchs = viso.getMatches();
            cout << "featureMatchs.size = " << featureMatchs.size() << endl;
            pose = viso.getMotion();
            cout << "pose=\n" << pose << endl;

        }

        waitKey(0);


    }

    return 0;
}

