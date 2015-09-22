//
// Created by 郭嘉丞 on 15/9/21.
//

#include "pointcloudUtilities.h"
#include <pcl/visualization/pcl_visualizer.h>

void rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

PointCloudPtr getSubXCloud(PointCloudPtr cloud, double fromX, double toX)
{
    PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (pcl::PointXYZRGB point: cloud->points)
    {
        if (point.x < toX && point.x > fromX)
        {
            newCloud->points.push_back(point);
        }
    }
    newCloud->width = (int) newCloud->points.size();
    newCloud->height = 1;
    return newCloud;
}

PointCloudPtr getSubYCloud(PointCloudPtr cloud, double fromY, double toY)
{
    PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (pcl::PointXYZRGB point: cloud->points)
    {
        if (point.y < toY && point.y > fromY)
        {
            newCloud->points.push_back(point);
        }
    }
    newCloud->width = (int) newCloud->points.size();
    newCloud->height = 1;
    return newCloud;
}

PointCloudPtr getSubZCloud(PointCloudPtr cloud, double fromZ, double toZ)
{
    PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (pcl::PointXYZRGB point: cloud->points)
    {
        if (point.z < toZ && point.z > fromZ)
        {
            newCloud->points.push_back(point);
        }
    }
    newCloud->width = (int) newCloud->points.size();
    newCloud->height = 1;
    return newCloud;
}

PointCloudPtr getSubCloud(PointCloudPtr cloud, double fromX, double toX,
                          double fromY, double toY, double fromZ, double toZ)
{
    PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (pcl::PointXYZRGB point: cloud->points)
    {
        if (point.x < toX && point.x > fromX
            && point.y < toY && point.y > fromY
            && point.z < toZ && point.z > fromZ)
        {
            newCloud->points.push_back(point);
        }
    }
    newCloud->width = (int) newCloud->points.size();
    newCloud->height = 1;
    return newCloud;
}