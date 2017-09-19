#ifndef TYPES_H_
#define TYPES_H_

#include <istream>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"


namespace POSE_GRAPH
{
struct Pose3d {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::map<int, Pose3d, std::less<int>,
Eigen::aligned_allocator<std::pair<const int, Pose3d> > >
MapOfPoses;

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Edge3d {
    int id_begin;
    int id_end;

    // The transformation that represents the pose of the end frame E w.r.t. the
    // begin frame B. In other words, it transforms a vector in the E frame to
    // the B frame.
    Pose3d t_be;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, z, delta orientation.
    Eigen::Matrix<double, 6, 6> information;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


typedef std::vector<Edge3d, Eigen::aligned_allocator<Edge3d> >
VectorOfEdges;

}



#endif  // TYPES_H_
