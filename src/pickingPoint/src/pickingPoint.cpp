#include <pickingPoint.hpp>
#include <cstdio>
#include <vector>
#include <cmath>
#include <limits>

PickingPoint::PickingPoint(const std::string& path)
    : path(path)
{
    m_Image = cv::imread(path, cv::IMREAD_COLOR);

    if(m_Image.empty())
    {
        perror("No image data\n");
        return;
    }
}

PickingPoint::PickingPoint(const cv::Mat& img)
    : m_Image(img), path("")
{
    if(m_Image.empty())
    {
        perror("No image data\n");
        return;
    }
}

cv::Point PickingPoint::Process()
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

    for(size_t i = 0; i < contours.size(); i++)
    {
        // Calculate the area of each contour
        double area = contourArea(contours[i]);

        // Get the rotated bounding box
        rect = cv::minAreaRect(contours[i]);
        cv::Point2f box[4];
        rect.points(box);

        // Retrieve the key parameters of the rotated bounding box
        cv::Point2f center = rect.center;
        int width = rect.size.width;
        int height = rect.size.height;
        float angle = rect.angle;

        if(width < height)
        {
            angle = 90 - angle;
        }
        else
        {
            angle = -angle;
        }

        #ifdef DRAWDEBUG
            // Draw the rectangle and label on the image
            cv::Point2f pt[4];
            rect.points(pt);
            for (int j = 0; j < 4; j++)
                cv::line(m_Image, pt[j], pt[(j + 1) % 4], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        #endif

        #ifdef DEBUG
            printf("Rotation angle %.2f\n", angle);
        #endif
    }

    cv::Mat M, rotated;

    // get angle and size from the bounding box
    float angle = rect.angle;
    cv::Size rect_size = rect.size;
    
    // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
    if (rect.angle < -45.0f) {
        angle += 90.0f;
        cv::swap(rect_size.width, rect_size.height);
    }
    // get the rotation matrix
    M = cv::getRotationMatrix2D(rect.center, angle, 1.0);

    // perform the affine transformation
    cv::warpAffine(m_Image, rotated, M, m_Image.size(), cv::INTER_CUBIC);

    // crop the resulting image
    cv::getRectSubPix(rotated, rect_size, rect.center, m_Cropped);

#ifdef DEBUG
    fprintf(stderr, "Extracting Cells...\n");
#endif

if (m_Cropped.rows == 0 || m_Cropped.cols == 0)
    return cv::Point(-1, -1);

    ExtractCells(5, m_Cropped);

#ifdef DEBUG
    fprintf(stderr, "Extracted Cells\n");
#endif

    for(int i = 0; i < m_Cells.size(); i++)
    {
        for(int j = 0; j < m_Cells[i].size(); j++)
        {
            HandleCell(m_Cells[i][j], i, j);
        }
    }

    std::pair<size_t, size_t> min_cell = FindMinCell();

    cv::Rect r = m_Cells[min_cell.first][min_cell.second].second;

#ifdef DEBUG
    fprintf(stderr, "Writing Image...\n");
#endif

    DrawHeatMap(path); 

    cv::Point pickPoint = cv::Point(r.x + r.width / 2, r.y + r.height / 2);

    cv::Point y0 = Raycast(pickPoint, cv::Point(0, 1)); // Sopra
    cv::Point y1 = Raycast(pickPoint, cv::Point(0, -1)); //sotto
    cv::Point x0 = Raycast(pickPoint, cv::Point(1, 0)); // destra
    cv::Point x1 = Raycast(pickPoint, cv::Point(-1, 0));    // Sinis

#ifdef DRAWDEBUG
    cv::circle(m_Cropped, y0, 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(m_Cropped, y1, 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(m_Cropped, x0, 2, cv::Scalar(255, 0, 0), -1);
    cv::circle(m_Cropped, x1, 2, cv::Scalar(255, 0, 0), -1);
#endif

    if (std::abs(y0.y - y1.y) > std::abs(x0.x - x1.x)) {
        pickPoint = cv::Point((x0.x + x1.x) / 2, pickPoint.y);
    } else {
        pickPoint = cv::Point(pickPoint.x, (y0.y + y1.y) / 2);
    }

    cv::circle(m_Cropped, pickPoint, 1, cv::Scalar(0, 0, 255), -1);

    cv::Mat M_inv, reverted;
    cv::Size original_size = m_Image.size();

    // Step 1: Create a new image of the original size
    cv::Mat new_image = cv::Mat::zeros(original_size, m_Cropped.type());

    // Get the image size
    int imgWidth = new_image.cols;
    int imgHeight = new_image.rows;

    // Calculate the ROI position and size
    int yValue = std::max(std::min(static_cast<int>(rect.center.y - rect_size.height / 2.0f), imgHeight - rect_size.height), 0);
    int xValue = std::max(std::min(static_cast<int>(rect.center.x - rect_size.width / 2.0f), imgWidth - rect_size.width), 0);

    // Step 2: Place m_Cropped in this new image at the position corresponding to the original rect
    cv::Rect original_rect(xValue, yValue, rect_size.width, rect_size.height);

    #ifdef DEBUG
        fprintf(stderr, "Original Rect: %d %d %d %d\n", original_rect.x, original_rect.y, original_rect.width, original_rect.height);
        fprintf(stderr, "Original Size: %d %d\n", original_size.width, original_size.height);
    #endif

    m_Cropped.copyTo(new_image(original_rect));

    // Step 3: Compute the inverse of M
    cv::invertAffineTransform(M, M_inv);

    // Step 4: Apply cv::warpAffine with the inverted matrix
    cv::warpAffine(new_image, reverted, M_inv, original_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Step 5: If the width and height were swapped, swap them back
    if (rect.angle < -45.0f) {
        cv::swap(reverted.cols, reverted.rows);
    }

    cv::Point newPickingPoint = FindColor(cv::Scalar(0, 0, 255), reverted);

    #ifdef DEBUG
        fprintf(stderr, "Picking Point: %d %d\n", newPickingPoint.x, newPickingPoint.y);
        
        cv::circle(reverted, newPickingPoint, 2, cv::Scalar(0, 255, 0), -1);
        cv::imwrite(std::string("output/") + std::strin g("result_") + path.substr(path.find_last_of("/") + 1), reverted);
        cv::imshow("Image", reverted);
        cv::imwrite(std::string("output/") + path.substr(path.find_last_of("/")), m_Cropped);

        fprintf(stderr, "Writed Image, Destroying Windows... \n");
        cv::waitKey(0); 
        cv::destroyAllWindows();

        fprintf(stderr, "Destroyed All Windows... \n");
    #endif

    return newPickingPoint;
}

cv::Point PickingPoint::FindColor(cv::Scalar color, cv::Mat& image) 
{
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
            
#ifdef DEBUG
            if(pixel[0] != 0 && pixel[1] != 0 && pixel[2] != 0)
                fprintf(stderr, "Color: %d %d %d\n", pixel[0], pixel[1], pixel[2]);
#endif
            
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
            #ifdef DEBUG
                fprintf(stderr, "Looping 1 %d %d %d %d %d\n", height - cell_size, height, cell_size, x, y);
            #endif
            int k = x*y + x;
            cv::Rect grid_rect(x, y, cell_size, cell_size);
            v.push_back({0, grid_rect});
            #ifdef DRAWDEBUG
                cv::rectangle(img, grid_rect, cv::Scalar(0, 255, 0), 1);
            #endif
        }

        if (width - x > 0) {
            int k = x*y + x;
            cv::Rect grid_rect(x, y, width - x, cell_size);
            v.push_back({0, grid_rect});
            #ifdef DRAWDEBUG
                cv::rectangle(img, grid_rect, cv::Scalar(0, 255, 0), 1);
            #endif
        }

        m_Cells.push_back(v);
    }

    if (height - y > 0) {
        int x;
        std::vector<std::pair<double, cv::Rect>> v;

        for (x = 0; x < width - cell_size; x += cell_size) {
            #ifdef DEBUG
                fprintf(stderr, "Looping 2\n");
            #endif
            int k = x*y + x;
            cv::Rect grid_rect(x, y, cell_size, height - y);
            v.push_back({0, grid_rect});
            #ifdef DRAWDEBUG
                cv::rectangle(img, grid_rect, cv::Scalar(0, 255, 0), 1);
            #endif
        }

        if (width - x > 0) {
            int k = x*y + x;
            cv::Rect grid_rect(x, y, width - x, height - y);
            v.push_back({0, grid_rect});
#ifdef DRAWDEBUG
            cv::rectangle(img, grid_rect, cv::Scalar(0, 255, 0), 1);
#endif

        }

        m_Cells.push_back(v);
    }
}

std::pair<size_t, size_t> PickingPoint::FindMaxCell()
{
    double min_value = std::numeric_limits<double>::min();
    size_t x, y; 

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

cv::Point PickingPoint::Raycast(cv::Point startingPoint, cv::Point direction) {
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

    while (currentPoint.x >= 0 && currentPoint.x < m_Cropped.cols && currentPoint.y >= 0 && currentPoint.y < m_Cropped.rows) {
        color = m_Cropped.at<cv::Vec3b>(currentPoint);

#ifdef DEBUG
        //fprintf(stderr, "Color: %d %d %d\n", color[0], color[1], color[2]);
#endif

        if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
            if(prev_color_black == false)
            {
                savedPoint = currentPoint;
            }
            prev_color_black = true;
            
#ifdef DEBUG
            //perror("Saved point\n");
#endif

        }
        else
        {
            prev_color_black = false;
        }

        currentPoint += direction;
    }

    if (!prev_color_black) {
#ifdef DEBUG
        fprintf(stderr, "Saved Point Outside\n");
#endif
        savedPoint = currentPoint;
    }
    
    return savedPoint;
}

void PickingPoint::DrawHeatMap(const std::string& path) {
    m_HeatMap = m_Cropped.clone();

    std::pair<size_t, size_t> max_cell = FindMaxCell();
    auto cell = m_Cells[max_cell.first][max_cell.second];
    double maxValue = cell.first;

    // pixelValue : maxValue = x : 255
    for(size_t i = 0; i < m_Cells.size(); i++)
    {
        for(size_t j = 0; j < m_Cells[i].size(); j++)
        {
            cv::rectangle(m_HeatMap, m_Cells[i][j].second, cv::Scalar(0, 0, (m_Cells[i][j].first * 255) / maxValue), -1);
        }
    }

#ifdef DEBUG
    fprintf(stderr, "Writing Heatmap... %s\n", (std::string("output/") + std::string("heatmap_") + path.substr(path.find_last_of("/") + 1)).c_str());
#endif
    //cv::imwrite(std::string("output/") + std::string("heatmap_") + path.substr(path.find_last_of("/") + 1), m_HeatMap);
}


// Non Ottimizzato: 4.572s
// Ottimizzato: 2.514s
void PickingPoint::HandleCell(std::pair<double, cv::Rect>& cell, int row, int col)
{
    if(GetPixelCount(m_Cells[row][col].second) == 0)
    {
        cell.first = std::numeric_limits<double>::max();
        return;
    }
    
    for(int i = 0; i < m_Cells.size(); i++)
    {
        for(int j = 0; j < m_Cells[i].size(); j++)
        {
            cell.first += GetPixelCount(m_Cells[i][j].second) * GetDistance(row, col, i, j);
        }
    }
}

double PickingPoint::GetDistance(int x1, int y1, int x2, int y2)
{
    if(x1 == x2)
        return std::abs(y1 - y2);
    if(y1 == y2)
        return std::abs(x1 - x2);

    return std::sqrt(std::pow(std::abs(x1 - x2), 2) + std::pow(std::abs(y1 - y2), 2)); //pitagora
}

unsigned int PickingPoint::GetPixelCount(cv::Rect& rect)
{
    unsigned int count = 0;

    for(int i = rect.y; i < rect.y + rect.height; i++)
    {
        for(int j = rect.x; j < rect.x + rect.width; j++)
        {
            cv::Vec3b color = m_Cropped.at<cv::Vec3b>(cv::Point(j, i));
            if(color[0] != 0 && color[1] != 0 && color[2] != 0)
            {
                count++;
            }
        }
    }

    return count;
}

std::pair<size_t, size_t> PickingPoint::FindMinCell()
{
    double min_value = std::numeric_limits<double>::max();
    size_t x, y; 

    for(size_t i = 0; i < m_Cells.size(); i++)
    {
        for(size_t j = 0; j < m_Cells[i].size(); j++)
        {
            if(m_Cells[i][j].first < min_value)
            {
                min_value = m_Cells[i][j].first;
                x = j;
                y = i;
            }
        }
    }

    return {y, x};
}