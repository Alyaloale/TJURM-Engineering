#ifndef MINING_TANK_GET_FOUR_H_
#define MINING_TANK_GET_FOUR_H_
#include "data_manager/param.h"
#include "data_manager/base.h"


struct MiningTankFour
{
    std::vector<std::vector<cv::Point2f>> point;//以三角形形式存储，顶点为第一个点，顺时针
    cv::Point2f center;
};


MiningTankFour GetMiningTankFour(std::vector<std::vector<cv::Point>> contours,cv::Mat& binary_image,cv::Mat& show_triangle); //获取四点
std::vector<std::vector<cv::Point>> MiningTankCountourFourSift(std::vector<std::vector<cv::Point>> &contours,cv::Mat& show_contour); //获取四点轮廓
#endif