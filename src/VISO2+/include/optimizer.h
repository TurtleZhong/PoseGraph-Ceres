#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "map.h"
#include "mappoint.h"
#include "frame.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


class optimizer
{
public:
    static int PoseOptimization(Frame &frame, Frame &lastframe);
};













#endif // OPTIMIZER_H
