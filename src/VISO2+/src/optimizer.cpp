#include "optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "converter.h"



int optimizer::PoseOptimization(Frame &frame, Frame& lastframe)
{


    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(frame.mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);


    // Set MapPoint vertices
    const int N = frame.N_total;
    //cout << GREEN"keyPoints = " << N << endl;


    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaStereo = sqrt(7.815);

    for(int i=0; i<N; i++)
    {
        MapPoint::Ptr pMP = frame.mvpMapPoints[i];

        if(pMP.get() != NULL)
        {
            nInitialCorrespondences++;


            frame.mvbOutlier[i] = false;


            //SET EDGE
            Eigen::Matrix<double,3,1> obs;
            const Matcher::maximum &kpUn = frame.mvfeatureCurrentLeft[i];
            //const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            //cout << pFrame->mnId << " : " << pFrame->mvKeys.size() << endl;
            const float &kp_ur = frame.mvuRight[i];
            obs << (double)kpUn.u, (double)kpUn.v, (double)kp_ur;



            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = 1;
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = frame.mpCamera->fx_;
            e->fy = frame.mpCamera->fy_;
            e->cx = frame.mpCamera->cx_;
            e->cy = frame.mpCamera->cy_;
            e->bf = frame.mpCamera->bf_;
            cout << "param: \n " << frame.mpCamera->fx_ << endl;
            cv::Mat Xw = pMP->GetWorldPos();
            e->Xw[0] = Xw.at<double>(0);
            e->Xw[1] = Xw.at<double>(1);
            e->Xw[2] = Xw.at<double>(2);

            cout << "3D p = " << Xw.at<double>(0) << " " << Xw.at<double>(1) << " " << Xw.at<double>(2) << endl;


            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);

            //cout << "vpEdgesStereo.size() = " << vpEdgesStereo.size() << endl;

        }

    }




    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    // const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={150,150,150,150}; // origin:10

    int nBad=0;

    for(size_t it=0; it<4; it++)
    {
        cout << "vpEdgesStereo.size() = " << vpEdgesStereo.size() << endl;

        vSE3->setEstimate(Converter::toSE3Quat(frame.mTcw));
        cout << "vSE3.estimate = \n" << frame.mTcw << endl;

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);
        nBad=0;
        //cin.get();


        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(frame.mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                frame.mvbOutlier[idx]=true; /*outliers*/
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                frame.mvbOutlier[idx]=false; /*inliers*/
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        cout << "optimizer.edges().size() = " << optimizer.edges().size() << endl;


        if(optimizer.edges().size()<10)
            break;
    }



    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    frame.setPose(pose);

    cout << BOLDGREEN << frame.mId << " : inliers = " << nInitialCorrespondences-nBad << endl;

    return nInitialCorrespondences-nBad;

}
