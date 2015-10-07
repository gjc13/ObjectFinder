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
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <map>

#include <set>
#include <algorithm>

#include "pointcloudUtilities.h"

using namespace std;


ProjectionDivider::ProjectionDivider(PointCloudPtr pointCloud, int minNumPoints)
        : originalCloud(pointCloud), yMin(0),
          minDividedNumPoints(minNumPoints)
{
    filteredCloud = removeBigPlanes(originalCloud, 0.2, 2, 5);
}

std::vector<PointCloudPtr> ProjectionDivider::getDividedPointClouds()
{
    vector<Hill> yDividePositions;
    vector<PointCloudPtr> dividedPointClouds;
    vector<PointCloudPtr> yDividedClouds = yDivide(yDividePositions);

    cout << "yPositions:" << endl;
    for (const Hill &hill:yDividePositions)
    {
        cout << hill.from << " " << hill.to << endl;
    }

//    int check;
//    cin >> check;
////    rgbVis(yDividedClouds[i]);
//    cout << endl;

    for (int i = 0; i < (unsigned) yDividePositions.size(); i++)
    {
        PointCloudPtr yDividedCloud = yDividedClouds[i];

        if (yDividedCloud == nullptr) continue;
        int xMin, zMin;
        cv::Mat xDensity = calculateDensity(0, xMin);
        cv::GaussianBlur(xDensity, xDensity, cv::Size(0, 0), 1);
        cv::Mat zDensity = calculateDensity(2, zMin);
        cv::GaussianBlur(zDensity, zDensity, cv::Size(0, 0), 2);
        vector<Hill> xDividePositions = getDividePositions(xDensity, 0.4, 0.05);
        vector<Hill> zDividePositions = getDividePositions(zDensity);

        addMin(xMin, xDividePositions);
        addMin(zMin, zDividePositions);

        cout << "xPositions:" << endl;
        for (const Hill &hill:xDividePositions)
        {
            cout << hill.from << " " << hill.to << endl;
        }

        cout << "zPositions:" << endl;
        for (const Hill &hill:zDividePositions)
        {
            cout << hill.from << " " << hill.to << endl;
        }

//        char buf[500];
//        sprintf(buf, "/Users/gjc13/KinectData/xDensity%d.txt", i);
//        saveDensity(buf, xDensity);
//        if (i == check)
//        {
//            getDividePositions(xDensity, 0.3, 0.05);
//            int xFrom, xTo;
//            cin >> xFrom >> xTo;
//            rgbVis(getSubXCloud(yDividedCloud, xFrom + xMin, xTo + xMin));
//        }


        cout << i << " " << yDividePositions[i].from << " " << yDividePositions[i].to << endl;
        cout << endl;

        map<GridInfo, PointCloudPtr> gridPointClouds;
        for (pcl::PointXYZRGB point: yDividedCloud->points)
        {
            int yDivideIndex = i;
            int zDivideIndex = getDivideIndex(point.z, zDividePositions);
            if (zDivideIndex < 0) continue;
            for (int j = 0; j < (int) xDividePositions.size(); j++)
            {
                if (point.x > xDividePositions[j].to || point.x < xDividePositions[j].from) continue;
                GridInfo gridInfo(j, yDivideIndex, zDivideIndex);
                if (gridPointClouds.find(gridInfo) == gridPointClouds.end())
                {
                    gridPointClouds[gridInfo] = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
                }
                gridPointClouds[gridInfo]->points.push_back(point);
            }
        }
//    GridInfo info(3, 2, 1);
//    rgbVis(gridPointClouds[info]);
        for (pair<GridInfo, PointCloudPtr> infoCloudPair : gridPointClouds)
        {
            cout << "grid:" << endl;
            cout << "x:" << infoCloudPair.first.indexX << " " << xDividePositions[infoCloudPair.first.indexX].from
            << " " << xDividePositions[infoCloudPair.first.indexX].to << endl;
            cout << "y:" << infoCloudPair.first.indexY << " " << yDividePositions[infoCloudPair.first.indexY].from
            << " " << yDividePositions[infoCloudPair.first.indexY].to << endl;
            cout << "z:" << infoCloudPair.first.indexZ << " " << zDividePositions[infoCloudPair.first.indexZ].from
            << " " << zDividePositions[infoCloudPair.first.indexZ].to << endl;
            cout << "numPoints" << infoCloudPair.second->points.size() << endl;
            if ((int) infoCloudPair.second->points.size() < minDividedNumPoints) continue;
            infoCloudPair.second->width = (int) infoCloudPair.second->points.size();
            infoCloudPair.second->height = 1;
            dividedPointClouds.push_back(infoCloudPair.second);
        }
    }
    return dividedPointClouds;
}

void ProjectionDivider::saveDensity(string filename, cv::Mat density)
{
    ofstream fout(filename);
    for (int i = 0; i < density.cols; i++)
    {
        fout << density.at<double>(i) << endl;
    }
    fout.close();
}

cv::Mat ProjectionDivider::calculateDensity(int projectionIndex, int &min)
{
    min = INT32_MAX;
    auto removePendingZeros = [](int *numPoints, cv::Mat &density) -> void {
        int end = 1000;
        while (end > 0 && numPoints[end - 1] == 0) end--;
        density = cv::Mat(1, end, CV_64FC1);
        for (int i = 0; i < end; i++)
        {
            density.at<double>(i) = numPoints[i];
        }
    };
    int numPoints[1000];
    for (int i = 0; i < 1000; i++)
    {
        numPoints[i] = 0;
    }
    for (pcl::PointXYZRGB point:filteredCloud->points)
    {
        if (point.data[projectionIndex] < min)
            min = (int) point.data[projectionIndex] - 1;
    }
    for (pcl::PointXYZRGB point:filteredCloud->points)
    {
        int distance = (int) (point.data[projectionIndex] - min);
        if (distance >= 1000) continue;
        numPoints[distance]++;
    }
    cv::Mat density;
    removePendingZeros(numPoints, density);
    return density;
}

std::vector<ProjectionDivider::Hill> ProjectionDivider::getDividePositions(cv::Mat density,
                                                                           double window_scale, double step_scale,
                                                                           double minWidth)
{
    set<Hill> hills;
    int windowLength = (int) (density.cols * window_scale);
    int stepLength = (int) (density.cols * step_scale);
    stepLength = stepLength == 0 ? 1 : stepLength;
    int start = 0;
    while (start + windowLength <= density.cols)
    {
        cv::Mat windowDensity(density, cv::Range::all(), cv::Range(start, start + windowLength));
        for (Hill &hill: getDividePositions(windowDensity))
        {
            hill.to += start;
            hill.from += start;
            hills.insert(hill);
        }
        start += stepLength;
    }
    vector<Hill> vecHills;
    for (const Hill &hill: hills)
    {
        vecHills.push_back(hill);
    }
    sort(vecHills.begin(), vecHills.end(), [](const Hill &lhs, const Hill &rhs) -> bool {
        return (lhs.to - lhs.from) < (rhs.to - rhs.from);
    });
    vector<Hill> finalHills;
    cout << "final Hills:" << endl;
    for (int i = 0; i < (int) hills.size(); i++)
    {
        if (vecHills[i].to - vecHills[i].from < minWidth) continue;
        bool isContained = false;
        for (int j = i + 1; j < (int) hills.size(); j++)
        {
            isContained = (vecHills[i].from >= vecHills[j].from && vecHills[i].to <= vecHills[j].to);
            if (isContained) break;
        }
        if (!isContained)
        {
            cout << vecHills[i].from << " " << vecHills[i].to << endl;
            finalHills.push_back(vecHills[i]);
        }
    }
    cout << endl;
    return finalHills;
}

vector<ProjectionDivider::Hill> ProjectionDivider::getDividePositions(cv::Mat density)
{
    vector<int> minPoints = getMinPoints(density);
    vector<Hill> dividePositions;
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
            dividePositions.push_back(Hill(hillStart, hillEnd));
        }
        else
        {
//            cout << "Dropped hill: (" << hillStart << "," << hillEnd << ")" << endl;
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
        cout << "plane removed" << endl;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(filteredCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);

        extract.filter(*cuttedCloud);
        filteredCloud = cuttedCloud;
    }
    return filteredCloud;
}


PointCloudPtr ProjectionDivider::removeColorRegion(PointCloudPtr cloud,
                                                   double sizeThreshold,
                                                   double distanceThreshold,
                                                   double colorThreshold)
{
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGB>);
    PointCloudPtr cuttedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> rgbGrower;
    rgbGrower.setInputCloud(cloud);
    rgbGrower.setSearchMethod(tree);
    rgbGrower.setDistanceThreshold((float) distanceThreshold);
    rgbGrower.setPointColorThreshold((float) colorThreshold);
    rgbGrower.setRegionColorThreshold(0);
    rgbGrower.setMinClusterSize((int) (cloud->size() * sizeThreshold));
    std::vector<pcl::PointIndices> clusters;
    rgbGrower.extract(clusters);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    for (pcl::PointIndices &indices: clusters)
    {
        cout << indices.indices.size();
        pcl::PointIndicesPtr indicesPtr(new pcl::PointIndices(indices));
        extract.setInputCloud(cloud);
        extract.setIndices(indicesPtr);
        extract.setNegative(true);
        extract.filter(*cuttedCloud);
        cloud = cuttedCloud;
    }
    return cloud;
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

int ProjectionDivider::getDivideIndex(double position, const std::vector<Hill> &dividePositions)
{
    int begin = 0, end = (int) dividePositions.size() - 1;
    while (begin <= end)
    {
        int mid = (begin + end) / 2;
        //divide start at pair first, end at pair second
        if (position >= dividePositions[mid].from && position <= dividePositions[mid].to)
            return mid;
        if (position <= dividePositions[mid].from)
            end = mid - 1;
        else
            begin = mid + 1;
    }
    return -1;
}

std::vector<PointCloudPtr> ProjectionDivider::yDivide(vector<Hill> &yDividePositions)
{
    cv::Mat yDensity = calculateDensity(1, yMin);
    cv::GaussianBlur(yDensity, yDensity, cv::Size(0, 0), 2);
    yDividePositions = getDividePositions(yDensity);
    vector<PointCloudPtr> dividedClouds(yDividePositions.size());
    for (PointCloudPtr &cloudPtr: dividedClouds)
    {
        cloudPtr = nullptr;
    }
    addMin(yMin, yDividePositions);
    for (pcl::PointXYZRGB point: filteredCloud->points)
    {
        int yDivideIndex = getDivideIndex(point.y, yDividePositions);
        if (yDivideIndex < 0) continue;
        if (dividedClouds[yDivideIndex] == nullptr)
        {
            dividedClouds[yDivideIndex] = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
        }
        dividedClouds[yDivideIndex]->points.push_back(point);
    }
    for (PointCloudPtr cloudPtr: dividedClouds)
    {
        if (cloudPtr != nullptr)
        {
            cloudPtr->width = (int) cloudPtr->points.size();
            cloudPtr->height = 1;
        }
    }
    return dividedClouds;
}

void ProjectionDivider::addMin(double min, std::vector<Hill> &dividePositions)
{
    for (Hill &hill:dividePositions)
    {
        hill.from += min;
        hill.to += min;
    }
}
