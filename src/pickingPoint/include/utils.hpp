#pragma once

#include <opencv2/opencv.hpp>

/**
 * \brief Extracts the pixels from the given image that are inside the given rectangle and saves them in the dst matrix
 * \param rr the rectangle to extract the pixels from
 * \param img the image to extract the pixels from
 * \param dst the image where the pixels will be saved
 */
extern cv::Mat GetRotRectImg(cv::RotatedRect rr,cv::Mat &img,cv::Mat& dst);

/**
 * \brief Reverts the operations applied by the GetRotRectImg function
 * \param src the image to revert the operations from
 * \param dst the image where the pixels will be saved
 * \param original_size the size of the original image
 * \param rect the rectangle that was used to extract the pixels
 * \param rect_size the size of the rectangle
 */
extern void RevertRotation(const cv::Mat& src, cv::Mat& dst, cv::Size original_size, cv::RotatedRect rect, cv::Size rect_size);