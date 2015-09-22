//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef OBJECTFINDER_PROJECTIONDIVIDER_H
#define OBJECTFINDER_PROJECTIONDIVIDER_H

#include <opencv2/opencv.hpp>
#include <utility>
#include "IPointCloudDivider.h"
#include "common.h"

class ProjectionDivider : public IPointCloudDivider
{
public:
    ProjectionDivider(PointCloudPtr pointCloud, int minNumPoints = 500);

    virtual std::vector<PointCloudPtr> getDividedPointClouds() override;

    void saveDensity();

    double getXMin()
    { return xMin; }

    double getYMin()
    { return yMin; }

    double getZMin()
    { return zMin; }

private:
    struct GridInfo
    {
        GridInfo(int idx, int idy, int idz)
                : indexX(idx), indexY(idy), indexZ(idz)
        { }

        int indexX;
        int indexY;
        int indexZ;

        bool operator<(const GridInfo &rhs) const
        {
            if (this->indexX == rhs.indexX)
            {
                if (this->indexY == rhs.indexY)
                    return this->indexZ < rhs.indexZ;
                return this->indexY < rhs.indexY;
            }
            return this->indexX < rhs.indexX;
        }
    };

    void calculateDensity();

    void filterDensity();

    std::vector<std::pair<int, int>> getDividePositions(cv::Mat density);

    std::vector<int> getMinPoints(cv::Mat density);

    std::pair<double, double> getMaxMin(cv::Mat density, int from, int to); //[from, to)

    PointCloudPtr removeBigPlanes(PointCloudPtr cloud, double sizeThreshold, double distanceThreshold, int retryTime);

    PointCloudPtr originalCloud;
    PointCloudPtr filteredCloud;
    cv::Mat xDensity;
    cv::Mat yDensity;
    cv::Mat zDensity;
    double xMin;
    double yMin;
    double zMin;
    int minDividedNumPoints;
};


#endif //OBJECTFINDER_PROJECTIONDIVIDER_H
