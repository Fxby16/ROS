
#include <pickingPoint.hpp>
#include <utils.hpp>
#include <cstdio>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

PickingPoint::PickingPoint(const std::string& path, const std::string& depth_path)
    : path(path)
{
    m_Image = cv::imread(path, cv::IMREAD_COLOR);
    m_DepthMap = cv::imread(depth_path, cv::IMREAD_UNCHANGED);

    if(m_Image.empty())
    {
        perror("No image data\n");
        return;
    }

    if(m_DepthMap.empty())
    {
        perror("No depth data\n");
        return;
    }
}

PickingPoint::PickingPoint(cv::Mat& image, cv::Mat& depth)
{
    m_Image = image.clone();
    m_DepthMap = depth.clone();
}

PickingPointInfo PickingPoint::Process()
{
    cv::Mat gray, binary;

    // Convert image to grayscale
    cv::cvtColor(m_Image, gray, cv::COLOR_BGR2GRAY);

    // Convert image to binary
    cv::threshold(gray, binary, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find all the contours in the thresholded image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(binary, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    cv::RotatedRect rect;

    double area = contourArea(contours[0]);

    // Get the rotated bounding box
    rect = cv::minAreaRect(contours[0]);
    cv::Point2f box[4];
    rect.points(box);

    cv::Mat M, rotated, rotated2;

    // get angle and size from the bounding box
    float angle = rect.angle;
    float requiredAngle = rect.angle;
    if (rect.size.width < rect.size.height) {
        requiredAngle += 90;
    }
    
    cv::Size rect_size = rect.size;
    
    // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
    if (rect.angle < -45.0f) {
        angle += 90.0f;
        cv::swap(rect_size.width, rect_size.height);
    }

    m_Cropped = cv::Mat(rect_size, m_Image.type());
    getRotRectImg(rect, m_Image, m_Cropped);

    m_DepthCropped = cv::Mat(rect_size, m_DepthMap.type());
    getRotRectImg(rect, m_DepthMap, m_DepthCropped);

    if (m_Cropped.rows == 0 || m_Cropped.cols == 0)
        return {cv::Point(-1, -1), -1.0};

    // Extract cells
    int cell_size = std::max((int) std::ceil(std::log10(m_Cropped.rows * m_Cropped.cols) * 2), 1);

    ExtractCells(cell_size, m_Cropped);

    // Iterate over all the cells to assign a score to each cell, in order to find the optimal cell
    for(int i = 0; i < m_Cells.size(); i++)
    {
        for(int j = 0; j < m_Cells[i].size(); j++)
        {
            HandleCell(m_Cells[i][j], i, j);
        }
    }

    //10 : 15 = cell_size : x
    size_t cells_to_get = std::max((8 * cell_size) / 10, 1);

    // Finding the first 10 cells with the lowest score
    std::vector<PickingPoint::Cell> min_cell_list = FindMinCell(1);

    cv::Rect r = m_Cells[min_cell_list[0].y][min_cell_list[0].x].second;
    
    cv::Point pickPoint = cv::Point(r.x + r.width / 2, r.y + r.height / 2);

    //cv::circle(Output, pickPoint, 2, cv::Scalar(255, 255, 255), -1);

    cv::Point y0 = Raycast(pickPoint, cv::Point(0, 1)); // up
    cv::Point y1 = Raycast(pickPoint, cv::Point(0, -1)); // down
    cv::Point x0 = Raycast(pickPoint, cv::Point(1, 0)); // right
    cv::Point x1 = Raycast(pickPoint, cv::Point(-1, 0)); // left

    //center the point on the shortest side
    if(std::abs(y0.y - y1.y) > std::abs(x0.x - x1.x)){
        pickPoint = cv::Point((x0.x + x1.x) / 2, pickPoint.y);
    }else{
        pickPoint = cv::Point(pickPoint.x, (y0.y + y1.y) / 2);
    }

    unsigned int requiredOpening1; // opening for the shortest side
    unsigned int requiredOpening2; // opening for the longest side
    float requiredAngle1 = requiredAngle; // angle for the shortest opening
    float requiredAngle2 = requiredAngle; // angle for the longest opening

    y0 = Raycast(pickPoint, cv::Point(0, 1), true); // Sopra
    y1 = Raycast(pickPoint, cv::Point(0, -1), true); //sotto
    x0 = Raycast(pickPoint, cv::Point(1, 0), true); // destra
    x1 = Raycast(pickPoint, cv::Point(-1, 0), true); // sinistra

    if(std::abs(y0.y - y1.y) > std::abs(x0.x - x1.x)){
        requiredOpening1 = std::abs(x0.x - x1.x) + 6;
        requiredOpening2 = std::abs(y0.y - y1.y) + 6;
        requiredAngle2 += 90;
        pickPoint = cv::Point((x0.x + x1.x) / 2, pickPoint.y);
    }else{
        requiredOpening1 = std::abs(y0.y - y1.y) + 6;
        requiredOpening2 = std::abs(x0.x - x1.x) + 6;
        requiredAngle1 += 90;
        pickPoint = cv::Point(pickPoint.x, (y0.y + y1.y) / 2);
    }

    //printf("Required Opening 1: %u Angle1: %f\nRequired Opening 2: %u Angle2: %f\n", requiredOpening1, requiredAngle1, requiredOpening2, requiredAngle2);

    cv::circle(m_Cropped, pickPoint, 1, cv::Scalar(0, 0, 255), -1);

    cv::Mat dstDepth;
    revertRotation(m_DepthCropped, dstDepth, m_Image.size(), rect, rect_size);
    
    cv::Mat dstImage;
    revertRotation(m_Cropped, dstImage, m_Image.size(), rect, rect_size);

    cv::Point newPickingPoint = FindColor(cv::Scalar(0, 0, 255), dstImage); //find the picking point on the original image

    //printf("New Picking Point: (%d, %d)\n", newPickingPoint.x, newPickingPoint.y);

    cv::circle(dstDepth, newPickingPoint, 1, cv::Scalar(0, 255, 0), -1);
    size_t count = 0;
    double avgDepth = 0.0;

    for(auto row : m_Cells)
    {
        for(auto cell : row)
        {
            double depth = GetAvgDepth(cell.second);

            if (depth <= 0) {
                count++;
                continue;
            }
            
            avgDepth += depth;
        }
    }



    //printf("[Prima] Value: %lf, avgDepth: %lf\n", (double) ((m_Cells.size() * m_Cells[0].size()) - count), avgDepth);

    avgDepth /= (double) ((m_Cells.size() * m_Cells[0].size()) - count);

    if (avgDepth <= 0 || avgDepth > 1000 || std::isnan(avgDepth) || std::isinf(avgDepth)) {
        avgDepth = 500;
    }

    PickingPointInfo info = {
        .point = newPickingPoint,
        .opening = {requiredOpening1, requiredOpening2},
        .angle = {requiredAngle1, requiredAngle2},
        .avgDepth = avgDepth
    };

    return info;
}


cv::Point PickingPoint::FindColor(cv::Scalar color, cv::Mat& image) 
{
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
            
            if (pixel[0] == color[0] && pixel[1] == color[1] && pixel[2] == color[2]) {
                return cv::Point(j, i);
            }
        }
    }

    return cv::Point(-1, -1);
}

void PickingPoint::ExtractCells(size_t cell_size, cv::Mat img)
{
    int width = img.cols;
    int height = img.rows;

    if (cell_size > width) {
        cell_size = width/2;
    }

    if (cell_size > height) {
        cell_size = height/2;
    }

    if (cell_size <= 0) {
        cell_size = 1;
    }

    int y;
    for (y = 0; y < height - cell_size; y += cell_size) {
        int x;

        std::vector<std::pair<double, cv::Rect>> v;

        for (x = 0; x < width - cell_size; x += cell_size) {
            int k = x*y + x;
            cv::Rect grid_rect(x, y, cell_size, cell_size);
            v.push_back({0, grid_rect});
        }

        if (width - x > 0) {
            int k = x*y + x;
            cv::Rect grid_rect(x, y, width - x, cell_size);
            v.push_back({0, grid_rect});
        }

        m_Cells.push_back(v);
    }

    if (height - y > 0) {
        int x;
        std::vector<std::pair<double, cv::Rect>> v;

        for (x = 0; x < width - cell_size; x += cell_size) {
            int k = x*y + x;
            cv::Rect grid_rect(x, y, cell_size, height - y);
            v.push_back({0, grid_rect});
        }

        if (width - x > 0) {
            int k = x*y + x;
            cv::Rect grid_rect(x, y, width - x, height - y);
            v.push_back({0, grid_rect});
        }

        m_Cells.push_back(v);
    }
    
    m_CellsCache.resize(m_Cells.size());
    m_DepthCache.resize(m_Cells.size());

    for(int i = 0; i < m_Cells.size(); i++)
    {
        m_CellsCache[i].assign(m_Cells[i].size(), std::numeric_limits<unsigned int>::max());
        m_DepthCache[i].assign(m_Cells[i].size(), std::numeric_limits<unsigned int>::max());
    }
}

std::pair<size_t, size_t> PickingPoint::FindMaxCell()
{
    double min_value = std::numeric_limits<double>::min();
    size_t x = 0, y = 0; 

    for(size_t i = 0; i < m_Cells.size(); i++)
    {
        for(size_t j = 0; j < m_Cells[i].size(); j++)
        {
            if(m_Cells[i][j].first > min_value && m_Cells[i][j].first != std::numeric_limits<double>::max())
            {
                min_value = m_Cells[i][j].first;
                x = j;
                y = i;
            }
        }
    }

    return {y, x};
}

cv::Point PickingPoint::Raycast(cv::Point startingPoint, cv::Point direction, bool useDepth) {
    cv::Point currentPoint = startingPoint;
    cv::Point savedPoint = startingPoint;

    bool prev_color_black;
    cv::Vec3b color = m_Cropped.at<cv::Vec3b>(currentPoint);

    if(color[0] == 0 && color[1] == 0 && color[2] == 0)
    {
        prev_color_black = true;
    }
    else
    {
        prev_color_black = false;
    }

    cv::Vec3f oldDepth = m_DepthCropped.at<cv::Vec3f>(currentPoint)[2];
    while (currentPoint.x >= 0 && currentPoint.x < m_Cropped.cols && currentPoint.y >= 0 && currentPoint.y < m_Cropped.rows) {
        color = m_Cropped.at<cv::Vec3b>(currentPoint);

        if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
            if(prev_color_black == false)
            {
                return currentPoint;
            }
            prev_color_black = true;
        }
        else
        {
            prev_color_black = false;
        }

        if (useDepth) {
            cv::Vec3f depth = m_DepthCropped.at<cv::Vec3f>(currentPoint)[2];
            if (std::abs(depth[2] - oldDepth[2]) > 30) {
                return savedPoint;
            }

            oldDepth = depth;
        }

        currentPoint += direction;
    }

    if (!prev_color_black) {
        savedPoint = currentPoint - direction;
    }
    
    return savedPoint;
}

// Non Ottimizzato: 4.572s
// Ottimizzato: 2.514s
void PickingPoint::HandleCell(std::pair<double, cv::Rect>& cell, int row, int col)
{
    if(GetPixelCount(m_Cells[row][col].second, row, col) == 0)
    {
        cell.first = std::numeric_limits<double>::max();
        return;
    }
    
    for(int i = 0; i < m_Cells.size(); i++)
    {
        for(int j = 0; j < m_Cells[i].size(); j++)
        {
            cell.first += GetPixelCount(m_Cells[i][j].second, i, j) * GetDistance(row, col, i, j);
        }
    }

    //cell.first /= std::max((double) GetMinDepth(m_Cells[row][col].second, row, col), 1.0);
}

unsigned int PickingPoint::GetMinDepth(cv::Rect& rect, size_t row, size_t col) {

    if(m_DepthCache[row][col] != std::numeric_limits<unsigned int>::max())
    {
        return m_DepthCache[row][col];
    }

    float min_val = std::numeric_limits<float>::max(), max_val = 0;

    for(int i = rect.y; i < rect.y + rect.height; i++)
    {
        for(int j = rect.x; j < rect.x + rect.width; j++)
        {
            float value = m_DepthCropped.at<cv::Vec3f>(i, j)[2];

            if(value < 250){
                value = 1000;
            }

            m_DepthCache[row][col] += value;
        }
    }

    m_DepthCache[row][col] /= (rect.width * rect.height);

    return m_DepthCache[row][col];
}

double PickingPoint::GetDistance(int x1, int y1, int x2, int y2)
{
    if(x1 == x2)
        return std::abs(y1 - y2);
    if(y1 == y2)
        return std::abs(x1 - x2);

    return std::sqrt(std::pow(std::abs(x1 - x2), 2) + std::pow(std::abs(y1 - y2), 2)); //pitagora
}

unsigned int PickingPoint::GetPixelCount(cv::Rect& rect, size_t row, size_t col)
{
    if(m_CellsCache[row][col] != std::numeric_limits<unsigned int>::max())
    {
        return m_CellsCache[row][col];
    }

    unsigned int count = 0;

    for(int i = rect.y; i < rect.y + rect.height; i++)
    {
        for(int j = rect.x; j < rect.x + rect.width; j++)
        {
            cv::Vec3b color = m_Cropped.at<cv::Vec3b>(i, j);
            if(color[0] != 0 && color[1] != 0 && color[2] != 0)
            {
                count++;
            }
        }
    }

    m_CellsCache[row][col] = count;

    return count;
}

std::vector<PickingPoint::Cell> PickingPoint::FindMinCell(unsigned int n)
{
    std::vector<PickingPoint::Cell> cells;

    for(int i = 0; i < m_Cells.size(); i++)
    {
        for(int j = 0; j < m_Cells[i].size(); j++)
        {
            cells.push_back({m_Cells[i][j].first, j, i});
        }
    }

    // Sort the cells based on their value
    std::sort(cells.begin(), cells.end(), [](const Cell& a, const Cell& b) {
        return a.value < b.value;
    });

    // Keep only the top n cells, if n is less than the size of cells
    if(n < cells.size())
    {
        cells.resize(n);
    }

    return cells;
}

std::pair<size_t, size_t> PickingPoint::GetCellFromPoint(cv::Point point, size_t cell_size)
{
    return {point.y / cell_size, point.x / cell_size};
}

std::vector<std::pair<size_t, size_t>> PickingPoint::GetCellsFromCenter(std::pair<size_t, size_t> center, size_t cells_to_get)
{
    int side_length = std::max(std::ceil(std::sqrt(cells_to_get)), 1.0);

    if(side_length % 2 == 0)
    {
        side_length++;
    }

    int start_x, end_x, start_y, end_y;

    start_x = std::max((int) center.second - side_length / 2, (int) 0);
    end_x = std::min(start_x + side_length, (int) m_Cells[0].size());

    start_y = std::max((int) center.first - side_length / 2, (int) 0);
    end_y = std::min(start_y + side_length, (int) m_Cells.size());

    std::vector<std::pair<size_t, size_t>> cells;

    for(size_t i = start_y; i < end_y; i++)
    {
        for(size_t j = start_x; j < end_x; j++)
        {
            cells.push_back({i, j});
        }
    }

    return cells;
}

double PickingPoint::GetAvgDepth(cv::Rect& rect)
{
    double sum = 0;
    size_t count = 0;

    for(int i = rect.y; i < rect.y + rect.height; i++)
    {
        for(int j = rect.x; j < rect.x + rect.width; j++)
        {
            if(m_DepthCropped.at<cv::Vec3f>(i, j)[2] < 260)
            {
                count++;
                continue;
            }

            sum += m_DepthCropped.at<cv::Vec3f>(i, j)[2];
        }
    }

    double avg = sum / (double) ((rect.width * rect.height) - count);

    //printf("Sum: %lf, Count: %lu, Questo: %lf, Rst: %lf\n", sum, count, (double) ((rect.width * rect.height) - count), avg);

    if (avg < 0 || std::isnan(avg)) {
        //printf("Avg: Nan!\n");
        return -1;
    }

    return avg;
}