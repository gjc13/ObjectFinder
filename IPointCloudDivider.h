//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef OBJECTFINDER_IPOINTCLOUDDIVIDER_H
#define OBJECTFINDER_IPOINTCLOUDDIVIDER_H

#include <vector>
#include "common.h"

class IPointCloudDivider
{
public:
    virtual std::vector<PointCloudPtr> getDividedPointClouds() = 0;
};


#endif //OBJECTFINDER_IPOINTCLOUDDIVIDER_H
