#ifndef MINING_TANK_GET_V_H_
#define MINING_TANK_GET_V_H_
#include "data_manager/param.h"
#include "data_manager/base.h"


struct MiningTankV
{
    std::vector<cv::Point2f> point;
};


std::vector<std::vector<cv::Point>> MiningTankCountourVSift(std::vector<std::vector<cv::Point>> &contours,cv::Mat& show_contour); //获取V字轮廓
MiningTankV GetMiningTankV(std::vector<std::vector<cv::Point>> contours,cv::Mat binary_image,cv::Mat& show_triangle); //获取V字
void StrenthenColor(cv::Mat &src, cv::Mat &dst, rm::ArmorColor color); //颜色增强
#endif