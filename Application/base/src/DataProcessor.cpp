// DataProcessor.cpp
#include "DataProcessor.h"
#include <iostream>
#include <iomanip>

DataProcessor::DataProcessor(int framesToAverage)
    : framesToAverage(framesToAverage)
{
}

void DataProcessor::addCentroid(const Eigen::Vector3d& centroid)
{
    centroids.push_back(centroid);

    if (centroids.size() >= static_cast<size_t>(framesToAverage))
    {
        computeStatistics();
        reset();
    }
}

void DataProcessor::reset()
{
    centroids.clear();
}

void DataProcessor::setFramesToAverage(int frames)
{
    framesToAverage = frames;
}

int DataProcessor::getFramesToAverage() const
{
    return framesToAverage;
}

Eigen::Vector3d DataProcessor::getMean() const
{
    return mean;
}

Eigen::Vector3d DataProcessor::getStdDev() const
{
    return stddev;
}

void DataProcessor::computeStatistics()
{
    if (centroids.empty())
        return;

    mean = Eigen::Vector3d::Zero();
    for (const auto& c : centroids)
        mean += c;
    mean /= static_cast<double>(centroids.size());

    Eigen::Vector3d variance = Eigen::Vector3d::Zero();
    for (const auto& c : centroids)
        variance += (c - mean).cwiseAbs2();
    variance /= static_cast<double>(centroids.size());

    stddev = variance.cwiseSqrt();
}
