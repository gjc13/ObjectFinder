//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef OBJECTFINDER_POINTCLOUDUTILITIES_H
#define OBJECTFINDER_POINTCLOUDUTILITIES_H

#include "common.h"

PointCloudPtr getSubXCloud(PointCloudPtr cloud, double fromX, double toX);

PointCloudPtr getSubYCloud(PointCloudPtr cloud, double fromY, double toY);

PointCloudPtr getSubZCloud(PointCloudPtr cloud, double fromZ, double toZ);

PointCloudPtr getSubCloud(PointCloudPtr cloud, double fromX, double toX,
                          double fromY, double toY, double fromZ, double toZ);

void rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

#endif //OBJECTFINDER_POINTCLOUDUTILITIES_H
