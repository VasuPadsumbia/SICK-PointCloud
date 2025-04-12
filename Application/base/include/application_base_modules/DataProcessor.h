// DataProcessor.h
#pragma once

#include <Eigen/Dense>
#include <vector>

class DataProcessor
{
public:
    DataProcessor(int framesToAverage = 50);

    void addCentroid(const Eigen::Vector3d& centroid);
    void reset();
    void setFramesToAverage(int frames);
    int getFramesToAverage() const;

    Eigen::Vector3d getMean() const;
    Eigen::Vector3d getStdDev() const;

private:
    void computeStatistics();

    std::vector<Eigen::Vector3d> centroids;
    int framesToAverage;
    Eigen::Vector3d mean;
    Eigen::Vector3d stddev;
};
