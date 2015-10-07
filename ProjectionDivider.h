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

    virtual std::vector<PointCloudPtr> getDividedPointClouds();

    void saveDensity(std::string filename, cv::Mat density);

    double getYMin()
    { return yMin; }

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

    struct Hill
    {
        Hill(const Hill & rhs)
            :from(rhs.from), to(rhs.to)
        {}

        Hill(int from_, int to_)
                : from(from_), to(to_)
        { }

        Hill & operator=(const Hill & rhs)
        {
            this->from = rhs.from;
            this->to = rhs.to;
            return *this;
        }

        int from;
        int to;

        bool operator<(const Hill &rhs) const
        {
            return this->from == rhs.from ? this->to < rhs.to : this->from < rhs.from;
        }
    };

    cv::Mat calculateDensity(int projectionIndex, int &min);

    std::vector<PointCloudPtr> yDivide(std::vector<Hill> &yDividePositions);

    int getDivideIndex(double position, const std::vector<Hill> &dividePositions);

    std::vector<Hill> getDividePositions(cv::Mat density);

    std::vector<Hill> getDividePositions(cv::Mat density,
                                         double window_scale, double step_scale, double minWidth = 5);

    std::vector<int> getMinPoints(cv::Mat density);

    std::pair<double, double> getMaxMin(cv::Mat density, int from, int to); //[from, to)

    void addMin(double min, std::vector<Hill> &dividePositions);

    PointCloudPtr removeBigPlanes(PointCloudPtr cloud,
                                  double sizeThreshold,
                                  double distanceThreshold,
                                  int retryTime);

    PointCloudPtr removeColorRegion(PointCloudPtr cloud,
                                    double sizeThreshold,
                                    double distanceThreshold,
                                    double colorThreshold);

    PointCloudPtr originalCloud;
    PointCloudPtr filteredCloud;


    int yMin;
    int minDividedNumPoints;
};


#endif //OBJECTFINDER_PROJECTIONDIVIDER_H
