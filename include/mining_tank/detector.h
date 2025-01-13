#ifndef MINING_TANK_DETECTOR_H_
#define MINING_TANK_DETECTOR_H_
#include "data_manager/param.h"
#include "data_manager/base.h"




void detect(cv::Mat &src);
std::vector<std::vector<cv::Point>> MiningTankCountourFourSift(std::vector<std::vector<cv::Point>> &contours); //获取四点轮廓
std::vector<std::vector<cv::Point>> MiningTankCountourVSift(std::vector<std::vector<cv::Point>> &contours); //获取V字轮廓
std::vector<std::pair<cv::Vec4d,cv::Vec4d>> GetMaxTwoLine(std::vector<std::vector<cv::Point2f>> contours,cv::Mat contour_image); //霍夫变换获取两条最长线段
MiningTankFour GetMiningTankFour(std::vector<std::vector<cv::Point>> contours,cv::Mat &binary_image); //获取四点
MiningTankV GetMiningTankV(std::vector<std::vector<cv::Point>> contours,cv::Mat binary_image); //获取V字
void StrenthenColor(cv::Mat &src, cv::Mat &dst, rm::ArmorColor color); //颜色增强
#endif