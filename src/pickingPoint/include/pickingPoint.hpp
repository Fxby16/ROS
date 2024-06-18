#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

class PickingPoint
{
public:

    PickingPoint(const std::string& path);
    PickingPoint(const cv::Mat& img);
    ~PickingPoint() = default;

    cv::Point Process();

    void ExtractCells(size_t cell_size, cv::Mat img);
    void HandleCell(std::pair<double, cv::Rect>& cell, int row, int col);
    double GetDistance(int x1, int y1, int x2, int y2);
    unsigned int GetPixelCount(cv::Rect& rect);
    void DrawHeatMap(const std::string& path);

    cv::Point Raycast(cv::Point startingPoint, cv::Point direction);
    cv::Point FindColor(cv::Scalar color, cv::Mat& image);

    std::pair<size_t, size_t> FindMinCell();
    std::pair<size_t, size_t> FindMaxCell();

private:
    cv::Mat m_Image;
    cv::Mat m_Cropped;
    cv::Mat m_HeatMap;

    std::string path;

    std::vector<std::vector<std::pair<double, cv::Rect>>> m_Cells;
};