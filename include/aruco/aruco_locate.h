#ifndef ARUCO_ARUCO_H
#define ARUCO_ARUCO_H_
#include "data_manager/param.h"
#include "data_manager/base.h"


void aruco_cereate();//生成ArUco标记
bool detectArucoMarkers(const cv::Mat& inputImage,
    std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f>>& markerCorners,
    bool drawMarkers , cv::Mat* outputImage );//检测ArUco标记



#endif