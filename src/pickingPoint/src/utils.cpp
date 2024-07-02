#include "opencv2/opencv.hpp"
#include <vector>
using namespace std;
using namespace cv;
//----------------------------------------------------------
// https://stackoverflow.com/a/26284491
//----------------------------------------------------------
void getQuadrangleSubPix_8u32f_CnR(const float *src, size_t src_step, Size src_size,
                                   float *dst, size_t dst_step, Size win_size,
                                   const double *matrix, int cn)
{
    int x, y, k;
    double A11 = matrix[0], A12 = matrix[1], A13 = matrix[2];
    double A21 = matrix[3], A22 = matrix[4], A23 = matrix[5];

    src_step /= sizeof(src[0]);
    dst_step /= sizeof(dst[0]);

    for (y = 0; y < win_size.height; y++, dst += dst_step)
    {
        double xs = A12 * y + A13;
        double ys = A22 * y + A23;

        for (x = 0; x < win_size.width; x++)
        {
            // Calculate nearest neighbor coordinates
            int ixs = cvRound(xs);
            int iys = cvRound(ys);
            const float *ptr = src + src_step * iys + ixs * cn;

            // Check bounds and assign nearest neighbor pixel values
            if (ixs >= 0 && iys >= 0 && ixs < src_size.width && iys < src_size.height)
            {
                for (int k = 0; k < cn; k++)
                {
                    dst[x * cn + k] = ptr[k];
                }
            }
            else
            {
                // Handle out-of-bounds by assigning a default value, e.g., 0
                for (int k = 0; k < cn; k++)
                {
                    dst[x * cn + k] = 0;
                }
            }

            xs += A11;
            ys += A21;
        }
    }
}

void getQuadrangleSubPix_8u32f_CnR(const uchar *src, size_t src_step, Size src_size,
                                   uchar *dst, size_t dst_step, Size win_size,
                                   const double *matrix, int cn)
{
    int x, y, k;
    double A11 = matrix[0], A12 = matrix[1], A13 = matrix[2];
    double A21 = matrix[3], A22 = matrix[4], A23 = matrix[5];

    src_step /= sizeof(src[0]);
    dst_step /= sizeof(dst[0]);

    for (y = 0; y < win_size.height; y++, dst += dst_step)
    {
        double xs = A12 * y + A13;
        double ys = A22 * y + A23;

        for (x = 0; x < win_size.width; x++)
        {
            // Calculate nearest neighbor coordinates
            int ixs = cvRound(xs);
            int iys = cvRound(ys);
            const uchar *ptr = src + src_step * iys + ixs * cn;

            // Check bounds and assign nearest neighbor pixel values
            if (ixs >= 0 && iys >= 0 && ixs < src_size.width && iys < src_size.height)
            {
                for (int k = 0; k < cn; k++)
                {
                    dst[x * cn + k] = ptr[k];
                }
            }
            else
            {
                // Handle out-of-bounds by assigning a default value, e.g., 0
                for (int k = 0; k < cn; k++)
                {
                    dst[x * cn + k] = 0;
                }
            }

            xs += A11;
            ys += A21;
        }
    }
}

//----------------------------------------------------------
//
//----------------------------------------------------------
cv::Mat myGetQuadrangleSubPix(const Mat &src, Mat &dst, Mat &m)
{
    CV_Assert(src.channels() == dst.channels());

    cv::Size win_size = dst.size();
    double matrix[6];
    cv::Mat M(2, 3, CV_64F, matrix);
    m.convertTo(M, CV_64F);
    double dx = (win_size.width - 1) * 0.5;
    double dy = (win_size.height - 1) * 0.5;
    matrix[2] -= matrix[0] * dx + matrix[1] * dy;
    matrix[5] -= matrix[3] * dx + matrix[4] * dy;

    if (src.type() == CV_32FC3) {
        getQuadrangleSubPix_8u32f_CnR((float *)src.data, src.step, src.size(),
                                        (float *)dst.data, dst.step, dst.size(),
                                        matrix, src.channels());
    } else {
        getQuadrangleSubPix_8u32f_CnR(src.data, src.step, src.size(),
                                        dst.data, dst.step, dst.size(),
                                        matrix, src.channels());
    }

    return M;
}
//----------------------------------------------------------
//
//----------------------------------------------------------
cv::Mat getRotRectImg(cv::RotatedRect rr, Mat &img, Mat &dst)
{
    Mat m(2, 3, CV_64FC1);
    float ang = rr.angle * CV_PI / 180.0;
    m.at<double>(0, 0) = cos(ang);
    m.at<double>(1, 0) = sin(ang);
    m.at<double>(0, 1) = -sin(ang);
    m.at<double>(1, 1) = cos(ang);
    m.at<double>(0, 2) = rr.center.x;
    m.at<double>(1, 2) = rr.center.y;
    return myGetQuadrangleSubPix(img, dst, m);
}

void revertRotation(const cv::Mat& src, cv::Mat& dst, cv::Size original_size, cv::RotatedRect rect, cv::Size rect_size) {
    cv::Mat reverted;

    // Step 1: Create a new image of the original size
    cv::Mat new_image = cv::Mat::zeros(original_size, src.type());

    // Get the image size
    int imgWidth = new_image.cols;
    int imgHeight = new_image.rows;

    // Calculate the ROI position and size
    int yValue = std::max(std::min(static_cast<int>(rect.center.y - rect_size.height / 2.0f), imgHeight - rect_size.height), 0);
    int xValue = std::max(std::min(static_cast<int>(rect.center.x - rect_size.width / 2.0f), imgWidth - rect_size.width), 0);

    // Step 2: Place m_Cropped in this new image at the position corresponding to the original rect
    cv::Rect original_rect(xValue, yValue, rect_size.width, rect_size.height);

    src.copyTo(new_image(original_rect));

    // Assuming you know the original angle of rotation
    double angle2 = rect.angle;

    // Calculate the center of the image around which we will rotate
    cv::Point2f center(original_rect.x + (original_rect.width / 2.0), original_rect.y + (original_rect.height / 2.0));

    // Calculate the rotation matrix for the negative angle (to rotate back)
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, -angle2, 1.0);

    cv::warpAffine(new_image, dst, rotationMatrix, new_image.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar());
}