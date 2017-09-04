#include "common_include.h"
#include "mappoint.h"


MapPoint::MapPoint()
    : mId(-1), mPos(Vector3d(0,0,0)), mNorm(Vector3d(0,0,0)), mbGood(true), mVisible_times(0), mMatched_times(0)
{

}


MapPoint::MapPoint (const Vector3d& position, const Vector3d& norm, unsigned long &LastFrameSeen, const vector<int32_t> &descriptor)
:  mPos(position), mNorm(norm), mnLastFrameSeen(LastFrameSeen), mbGood(true), mVisible_times(1), mMatched_times(1), mvDescriptor(descriptor)
{

}


unsigned long MapPoint::mFactory_id = 0;
