#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include "common.h"
#include "ProjectionDivider.h"
#include "pointcloudUtilities.h"
#include <cstdio>

using namespace std;

int main()
{
    PointCloudPtr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile("/Users/gjc13/KinectData/pointCloud.pcd", *pointCloud) == -1)
    {
        PCL_ERROR("cannot load point cloud");
        return -1;
    }
    ProjectionDivider divider(pointCloud);
    vector<PointCloudPtr> dividedPointClouds = divider.getDividedPointClouds();
    cout << dividedPointClouds.size() << endl;
    char buffer[20];
    for (int i = 0; i < (int) dividedPointClouds.size(); i++)
    {
        string prefix = "/Users/gjc13/KinectData/Divisions/divideCloud";
        sprintf(buffer, "%d", i);
        pcl::io::savePCDFile(prefix + buffer + ".pcd", *dividedPointClouds[i], true);
    }
    return 0;
}