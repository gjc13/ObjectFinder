#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include "common.h"
#include "ProjectionDivider.h"
#include "pointcloudUtilities.h"
#include "ImageRebuild.h"
#include <cstdio>

using namespace std;

int main()
{
    PointCloudPtr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    cv::Mat totalImage = cv::imread("/Users/gjc13/KinectData/rgb.png");
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
        string pcdPrefix = "/Users/gjc13/KinectData/dividedSamples/divideCloud";
        string lowResPrefix = "/Users/gjc13/KinectData/dividedSamples/lowRes";
        string highResPrefix = "/Users/gjc13/KinectData/dividedSamples/highRes";
        sprintf(buffer, "%d", i);
        pcl::io::savePCDFile(pcdPrefix + buffer + ".pcd", *dividedPointClouds[i], true);
        cv::imwrite(lowResPrefix + buffer + ".png", get2DImageFromPointCloud(dividedPointClouds[i]));
        cv::imwrite(highResPrefix + buffer + ".png", getHDImageFromPointCloud(dividedPointClouds[i], totalImage));
    }
    return 0;
}