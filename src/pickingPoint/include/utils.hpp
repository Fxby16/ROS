#pragma once

#include <opencv2/opencv.hpp>

extern cv::Mat getRotRectImg(cv::RotatedRect rr,cv::Mat &img,cv::Mat& dst);
extern void revertRotation(const cv::Mat& src, cv::Mat& dst, cv::Size original_size, cv::RotatedRect rect, cv::Size rect_size);