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

    /**
     * \brief Main function that processes the given mask and depth images and finds the picking point
     */
    PickingPointInfo Process();

    /**
     * \brief Extracts the cells from the given image and save them in the m_Cells matrix
     * \param cell_size the size of the cell
     * \param img the image to extract the cells from
     */
    void ExtractCells(size_t cell_size, cv::Mat img);

    /**
     * \brief Iterates over the cells and gives each cell a score based on the cells around it
     * \param cell the cell to handle
     * \param row the row of the cell
     * \param col the column of the cell
     */
    void HandleCell(std::pair<double, cv::Rect>& cell, int row, int col);

    /**
     * \brief Returns the distance between two points
     */
    double GetDistance(int x1, int y1, int x2, int y2);

    /**
     * \brief Returns the number of non-black pixels in the given rectangle (it samples the m_Cropped image)
     * \param rect the rectangle to check
     * \param row the row of the cell
     * \param col the column of the cell
     */
    unsigned int GetPixelCount(cv::Rect& rect, size_t row, size_t col);

    /**
     * \brief Calculates the average pixel depth of the given rectangle (it samples the m_DepthCropped image)
     * \param rect the rectangle to check
     * \return the average depth of the rectangle
     */
    double GetAvgDepth(cv::Rect& rect);

    /**
     * \return the indices of the cell that contains the given point (y, x)
    */
    std::pair<size_t, size_t> GetCellFromPoint(cv::Point point, size_t cell_size);

    /**
     * \return the indices of the cells around the given center cell
    */
    std::vector<std::pair<size_t, size_t>> GetCellsFromCenter(std::pair<size_t, size_t> center, size_t cells_to_get);

    /**
     * \return the point where the ray intersects the first non-black pixel or a big difference in depth
     */
    cv::Point Raycast(cv::Point startingPoint, cv::Point direction, bool useDepth = false);

    /**
     * \brief Finds the color in the image and returns the point where it was found
     * \param color the color to find
     * \param image the image to search in
     * \return the point where the color was found
     */
    cv::Point FindColor(cv::Scalar color, cv::Mat& image);

    /**
     * \return the first n cells with the lowest score assigned by HandleCell
     */
    std::vector<Cell> FindMinCell(unsigned int n);

    /**
     * \return the cell with the highest score assigned by HandleCell
     */
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