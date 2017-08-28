#include "common_include.h"
#include "mappoint.h"


MapPoint::MapPoint()
    : mId(-1), mPos(Vector3d(0,0,0)), mNorm(Vector3d(0,0,0)), mbGood(true), mVisible_times(0), mMatched_times(0)
{

}


MapPoint::MapPoint (long id, const Vector3d& position, const Vector3d& norm, Frame* frame, const vector<int32_t> &descriptor)
: mId(id), mPos(position), mNorm(norm), mbGood(true), mVisible_times(1), mMatched_times(1), mvDescriptor(descriptor)
{
    mObserved_frames.push_back(frame);
}

MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr(
        new MapPoint( mFactory_id++, Vector3d(0,0,0), Vector3d(0,0,0) )
    );
}


MapPoint::Ptr MapPoint::createMapPoint(const Eigen::Vector3d &pos_world,
                                       const Eigen::Vector3d &Norm,
                                       const vector<int32_t> &Descriptor,
                                       Frame *frame)
{

    return MapPoint::Ptr(
                new MapPoint(mFactory_id++, pos_world, Norm, frame, Descriptor)
                );
}

unsigned long MapPoint::mFactory_id = 0;
