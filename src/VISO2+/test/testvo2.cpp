#include "common_include.h"
#include "config.h"
#include "tracking.h"
#include "viso_stereo.h"
#include "camera.h"
#include "frame.h"
#include "fstream"



using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{

    string dir = "/home/m/KITTI/dataset/sequences/01";


    Config::setParameterFile("/home/m/ws_orb2/src/VISO2+/config/KITTI00-02.yaml");
    Camera* camera (new Camera);

    VisualOdometryStereo::parameters param;
    param.calib.f = camera->fx_;
    param.calib.cu = camera->cx_;
    param.calib.cv = camera->cy_;
    param.base = camera->b_;

    VisualOdometryStereo viso(param);
    //Tracking Tracker;
    Tracking* Tracker = new Tracking();

    //Matrix pose = Matrix::eye(4);

    Frame lastFrame;

    ofstream outFile;
    outFile.open("../camera_pose.txt");
    Mat pose = Mat::eye(4,4,CV_64FC1);


    for(int32_t i = 0; i < 1100; i++)
    {
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        Mat Left = imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        Mat Right = imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);

        Frame currentFrame(i,camera,Left,Right);




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


        // compute visual odometry
        int32_t dims[] = {width,height,width};


        cout << "Process :" << i << endl;
        Tracker->generateFrame(i,camera,Left,Right);

        currentFrame = Frame(Tracker->curr_);


        //cout << "lastframe" << endl << lastFrame.mId << endl;
        cout << "currentFrame" << endl << Tracker->curr_.mId << endl;

        //cout << BOLDYELLOW"lastframe: pose\n" << lastFrame.mTcw << endl;
        cout << BOLDYELLOW"currframe: pose\n" << currentFrame.mvCircularMatches.size() << endl;
        cout << BOLDCYAN"stereo matchsize" << currentFrame.N_parallel << endl;

        pose = pose * currentFrame.mTcw.inv();
        cout << BOLDGREEN"pose = \n " << pose << endl;

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(i==2 && j==3)
                    outFile << pose.at<double>(i,j);
                else
                    outFile << pose.at<double>(i,j) << " ";

            }
        }
        outFile << endl;


        /*to make sure the depth and descriptor is correct !!! --> very important*/
        std::vector<Matcher::p_match> left = currentFrame.getStereoMatches();
        std::vector<Matcher::maximum> desp = currentFrame.mvfeatureCurrentLeft;

        if(currentFrame.mId > 3)
        {
            for(int i = 0; i < 10; i++)
            {
                cout << left[i].i1c << " depth = " << currentFrame.mpCamera->bf_ / (left[i].u1c - left[i].u2c) << " " << "u= " << left[i].u1c << " v= " << left[i].v1c << "  ";

                cout << "desp[left[i].i1c].u = " << desp[left[i].i1c].u << " desp[left[i].i1c].v = " << desp[left[i].i1c].v << " ";

                cout << "mvDepth[" << left[i].i1c << "] = " << currentFrame.mvDepth[left[i].i1c] << endl;
            }
        }


        //lastFrame = Frame(currentFrame);


        //cout << "!!!!!!!!!" << Tracker->curr_.mTcw << endl;
//        cout << "features = \n" << Tracker->curr_.N_total << endl;
//        cout << "lalallal\n" << Tracker->curr_.N_circular << endl;

//        //Tracker->viso_->process(currentFrame.mpimg_left_data,currentFrame.mpimg_right_data,currentFrame.dims);

//        cout << BOLDGREEN"pose=\n" << Tracker->viso_->getMotion() << endl;
        //currentFrame.mpimg_left_data,currentFrame.mpimg_right_data,dims
        //left_img_data,right_img_data,dims

//        if(viso.process(currentFrame.mpimg_left_data,currentFrame.mpimg_right_data,dims))
//        {

//            std::vector<Matcher::p_match> featureMatchs = viso.getMatches();
//            cout << "featureMatchs.size = " << featureMatchs.size() << endl;
//            pose = viso.getMotion();
//            cout << "pose=\n" << pose << endl;
//            cout << endl << "new test " << endl;

//            cout << endl << "current left =" << viso.matcher->mvfeatureCurrentLeft[1].size() << endl;
//            cout << endl << "current right = " << viso.matcher->mvfeatureCurrentRight.size() << endl;
//            cout << endl << "previous left =" << viso.matcher->mvfeaturePreviousLeft[1].size() << endl;
//            cout << endl << "previous right = " << viso.matcher->mvfeaturePreviousRight.size() << endl;
//            cout << "$$$$pose: " << endl << viso.getMatches().size() << " " << viso.getMotion() << endl;

//            //cout << "featuresize = " << currentFrame.mpMatcher->getFeaturePrevious().size() << endl;
////            std::vector <std::vector<Matcher::maximum>> featurePrevious = currentFrame.mpMatcher->getFeaturePrevious();
////            std::vector<Matcher::maximum> sparseFeatures = featurePrevious[0];
////            cout << "sparse feature = " << sparseFeatures.size() << endl;
//            imshow("currentFrame",currentFrame.mImgLeft);
//            cv::waitKey(0);

//        }

        imshow("currentFrame",currentFrame.mImgLeft);
        cv::waitKey(27);





    }

    return 0;
}

