#pragma once

#include <vector>
#include <opencv2/opencv.hpp>




double EllipseOverlap(cv::RotatedRect &ellipse1, cv::RotatedRect &ellipse2);


void EllipseNonMaximumSuppression(std::vector<cv::RotatedRect> &detElps, std::vector<double> &detEllipseScore, double T_iou);
