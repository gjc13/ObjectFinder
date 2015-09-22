//
// Created by 郭嘉丞 on 15/9/21.
//

#include "ProjectionDivider.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <map>

using namespace std;


ProjectionDivider::ProjectionDivider(PointCloudPtr pointCloud, int minNumPoints)
        : originalCloud(pointCloud),
          minDividedNumPoints(minNumPoints)
{
    filteredCloud = removeBigPlanes(originalCloud, 0.3, 2, 5);
    calculateDensity();
    filterDensity();
    saveDensity();
}

std::vector<PointCloudPtr> ProjectionDivider::getDividedPointClouds()
{
    vector<pair<int, int>> xDividePositions = getDividePositions(xDensity);
    vector<pair<int, int>> yDividePositions = getDividePositions(yDensity);
    vector<pair<int, int>> zDividePositions = getDividePositions(zDensity);
    auto addMin = [](double min, vector<pair<int, int>> &dividePositions) -> void {
        for (pair<int, int> &posPair:dividePositions)
        {
            posPair.first += min;
            posPair.second += min;
        }
    };
    addMin(xMin, xDividePositions);
    addMin(yMin, yDividePositions);
    addMin(zMin, zDividePositions);
    map<GridInfo, PointCloudPtr> gridPointClouds;
    auto getDivideIndex = [](double position, const vector<pair<int, int>> &dividePositions) -> int {
        int begin = 0, end = (int) dividePositions.size() - 1;
        while (begin <= end)
        {
            int mid = (begin + end) / 2;
            //divide start at pair first, end at pair second
            if (position >= dividePositions[mid].first && position <= dividePositions[mid].second)
                return mid;
            if (position <= dividePositions[mid].first)
                end = mid - 1;
            else
                begin = mid + 1;
        }
        return -1;
    };
    for (pcl::PointXYZRGB point: filteredCloud->points)
    {
        int xDivideIndex = getDivideIndex(point.x, xDividePositions);
        if (xDivideIndex < 0) continue;
        int yDivideIndex = getDivideIndex(point.y, yDividePositions);
        if (yDivideIndex < 0) continue;
        int zDivideIndex = getDivideIndex(point.z, zDividePositions);
        if (zDivideIndex < 0) continue;
        GridInfo gridInfo(xDivideIndex, yDivideIndex, zDivideIndex);
        if (gridPointClouds.find(gridInfo) == gridPointClouds.end())
        {
            gridPointClouds[gridInfo] = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
        }
        gridPointClouds[gridInfo]->points.push_back(point);
    }
    vector<PointCloudPtr> dividedPointClouds;
    for (pair<GridInfo, PointCloudPtr> infoCloudPair : gridPointClouds)
    {
        if ((int) infoCloudPair.second->points.size() < minDividedNumPoints) continue;
        infoCloudPair.second->width = (int) infoCloudPair.second->points.size();
        infoCloudPair.second->height = 1;
        dividedPointClouds.push_back(infoCloudPair.second);
    }
    return dividedPointClouds;
}

void ProjectionDivider::saveDensity()
{
    ofstream fout("/Users/gjc13/ClionProjects/KinectToPCL/xdensity_d.txt");
    for (int i = 0; i < xDensity.cols; i++)
    {
        fout << xDensity.at<double>(i) << endl;
    }
    fout.close();
    fout.open("/Users/gjc13/ClionProjects/KinectToPCL/ydensity_d.txt");
    for (int i = 0; i < yDensity.cols; i++)
    {
        fout << yDensity.at<double>(i) << endl;
    }
    fout.close();
    fout.open("/Users/gjc13/ClionProjects/KinectToPCL/zdensity_d.txt");
    for (int i = 0; i < zDensity.cols; i++)
    {
        fout << zDensity.at<double>(i) << endl;
    }
    fout.close();
}

void ProjectionDivider::calculateDensity()
{
    xMin = INT32_MAX;
    yMin = INT32_MAX;
    zMin = INT32_MAX;
    auto removePendingZeros = [](int *numPoints, cv::Mat &density) -> void {
        int end = 1000;
        while (end > 0 && numPoints[end - 1] == 0) end--;
        density = cv::Mat(1, end, CV_64FC1);
        for (int i = 0; i < end; i++)
        {
            density.at<double>(i) = numPoints[i];
        }
    };
    int xNumPoints[1000], yNumPoints[1000], zNumPoints[1000];
    for (int i = 0; i < 1000; i++)
    {
        xNumPoints[i] = 0;
        yNumPoints[i] = 0;
        zNumPoints[i] = 0;
    }
    for (pcl::PointXYZRGB point:filteredCloud->points)
    {
        if (point.x < xMin) xMin = point.x;
        if (point.y < yMin) yMin = point.y;
        if (point.z < zMin) zMin = point.z;
    }
    for (pcl::PointXYZRGB point:filteredCloud->points)
    {
        int x = (int) (point.x - xMin);
        int y = (int) (point.y - yMin);
        int z = (int) (point.z - zMin);
        if (x > 1000 || y > 1000 || z > 1000) continue;
        xNumPoints[x]++;
        yNumPoints[y]++;
        zNumPoints[z]++;
    }

    removePendingZeros(xNumPoints, xDensity);
    removePendingZeros(yNumPoints, yDensity);
    removePendingZeros(zNumPoints, zDensity);
}

void ProjectionDivider::filterDensity()
{
    cv::GaussianBlur(xDensity, xDensity, cv::Size(0, 0), 1);
    cv::GaussianBlur(yDensity, yDensity, cv::Size(0, 0), 2);
    cv::GaussianBlur(zDensity, zDensity, cv::Size(0, 0), 2);
}

vector<pair<int, int>> ProjectionDivider::getDividePositions(cv::Mat density)
{
    vector<int> minPoints = getMinPoints(density);
    vector<pair<int, int>> dividePositions;
    pair<double, double> totalMaxMin = getMaxMin(density, 0, density.cols);
    double range = totalMaxMin.first - totalMaxMin.second;
    int hillStart;
    int hillEnd;
    double hillRange = INT32_MAX;
    for (int i = 0; i < (int) minPoints.size() - 1; i++)
    {
        hillStart = minPoints[i];
        hillEnd = minPoints[i + 1];
        pair<double, double> hillMaxMin = getMaxMin(density, hillStart, hillEnd + 1);
        hillRange = hillMaxMin.first - hillMaxMin.second;
        if (hillRange > range / 20)
        {
            dividePositions.push_back(pair<int, int>(hillStart, hillEnd));
        }
    }
    return dividePositions;
}

vector<int> ProjectionDivider::getMinPoints(cv::Mat density)
{
    double threshold = cv::mean(density)[0] * 1.5;
    vector<int> possibleMinPoints;
    for (int i = 1; i < density.cols - 1; i++)
    {
        if (density.at<double>(i) >= threshold) continue;
        int nowIndex = i + 1;
        bool isMinPoint = true;
        while (nowIndex < density.cols)
        {
            if (density.at<double>(i) < density.at<double>(nowIndex))
                break;
            else if (density.at<double>(i) == density.at<double>(nowIndex))
                nowIndex++;
            else
            {
                isMinPoint = false;
                break;
            }
        }
        if (!isMinPoint) continue;
        nowIndex = i - 1;
        while (nowIndex >= 0)
        {
            if (density.at<double>(i) < density.at<double>(nowIndex))
                break;
            else if (density.at<double>(i) == density.at<double>(nowIndex))
                nowIndex--;
            else
            {
                isMinPoint = false;
                break;
            }
        }
        if (isMinPoint) possibleMinPoints.push_back(i);
    }
    bool isConsecutive = false;
    vector<int> minPoints;
    minPoints.push_back(0);
    int lastIndex = -2;
    int startIndex = 0;
    for (int index:possibleMinPoints)
    {
        if (index == lastIndex + 1)
            isConsecutive = true;
        else
        {
            if (isConsecutive)
                minPoints.push_back((startIndex + lastIndex) / 2);
            else if (lastIndex > 0) minPoints.push_back(lastIndex);
            isConsecutive = false;
            startIndex = index;
        }
        lastIndex = index;
    }
    if (isConsecutive)
    {
        minPoints.push_back((startIndex + lastIndex) / 2);
    }
    else
    {
        minPoints.push_back(lastIndex);
    }
    minPoints.push_back(density.cols - 1);
    return minPoints;
}

PointCloudPtr ProjectionDivider::removeBigPlanes(PointCloudPtr cloud, double sizeThreshold, double distanceThreshold,
                                                 int retryTime)
{
    PointCloudPtr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    PointCloudPtr cuttedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    filteredCloud->points = cloud->points;
    filteredCloud->height = cloud->height;
    filteredCloud->width = cloud->width;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    //TODO:test if we should optimize the coefficients here
    //see http://answers.ros.org/question/55223/what-does-setoptimizecoefficients-do-in-sacsegmentation/
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    //TODO:read introduction of other method types and decide whether they are better
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(distanceThreshold); //2cm
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients coefficients;
    int timeUnfound = 0;
    int minPlanePoints = (int) (cloud->points.size() * sizeThreshold);
    minPlanePoints = minPlanePoints <= 20 ? 20 : minPlanePoints;
    while (timeUnfound < retryTime)
    {
        seg.setInputCloud(filteredCloud);
        seg.segment(*inliers, coefficients);
        if (inliers->indices.size() < minPlanePoints)
        {
            timeUnfound++;
            continue;
        }

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(filteredCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);

        extract.filter(*cuttedCloud);
        filteredCloud = cuttedCloud;
    }
    return filteredCloud;
}

pair<double, double> ProjectionDivider::getMaxMin(cv::Mat density, int from, int to)
{
    double max = -1;
    double min = INT32_MAX;
    for (int j = from; j < to; j++)
    {
        if (density.at<double>(j) > max) max = density.at<double>(j);
        if (density.at<double>(j) < min) min = density.at<double>(j);
    }
    return pair<double, double>(max, min);
}
