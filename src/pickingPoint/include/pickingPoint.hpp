#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

struct PickingPointInfo{
    cv::Point point;
    double opening[2];
    double angle[2];
    double avgDepth;
};

class PickingPoint
{
public:
    PickingPoint(const std::string& mask_path, const std::string& depth_path, const std::string& full_rgb_path);
    PickingPoint(cv::Mat& mask, cv::Mat& depth, cv::Mat& full_rgb);
    ~PickingPoint() = default;

    struct Cell
    {
        double value;
        unsigned int x;
        unsigned int y;
    };

    PickingPointInfo Process();

    void ExtractCells(size_t cell_size, cv::Mat img);
    void HandleCell(std::pair<double, cv::Rect>& cell, int row, int col);
    double GetDistance(int x1, int y1, int x2, int y2);
    unsigned int GetPixelCount(cv::Rect& rect, size_t row, size_t col);
    unsigned int GetMinDepth(cv::Rect& rect, size_t row, size_t col);
    double GetAvgDepth(cv::Rect& rect);

    /**
     * \return the indices of the cell that contains the given point (y, x)
    */
    std::pair<size_t, size_t> GetCellFromPoint(cv::Point point, size_t cell_size);

    std::vector<std::pair<size_t, size_t>> GetCellsFromCenter(std::pair<size_t, size_t> center, size_t cells_to_get);

    cv::Point Raycast(cv::Point startingPoint, cv::Point direction, bool useDepth = false);
    cv::Point FindColor(cv::Scalar color, cv::Mat& image);

    std::vector<Cell> FindMinCell(unsigned int n);
    std::pair<size_t, size_t> FindMaxCell();

private:
    cv::Mat m_Image;
    cv::Mat m_Cropped;
    cv::Mat m_HeatMap;
    cv::Mat m_DepthMap;
    cv::Mat m_DepthCropped;
    cv::Mat m_DepthCroppedNormalized;

    cv::Mat m_FullRGB;

    std::vector<std::vector<std::pair<double, cv::Rect>>> m_Cells;
    std::vector<std::vector<unsigned int>> m_CellsCache;
    std::vector<std::vector<unsigned int>> m_DepthCache;

    std::string path="";
};