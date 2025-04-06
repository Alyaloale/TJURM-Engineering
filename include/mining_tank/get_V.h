#ifndef MINING_TANK_GET_V_H_
#define MINING_TANK_GET_V_H_
#include "data_manager/param.h"
#include "data_manager/base.h"


void MiningTankCountourVSift(std::vector<std::vector<cv::Point>> contours,cv::Mat& show_contour, std::vector<std::vector<cv::Point>>* mining_tank_contours); //获取V字轮廓
void GetMiningTankV(std::vector<std::vector<cv::Point>>* contours,cv::Mat &binary_image,cv::Mat& show_triangle, std::vector<cv::Point2f>* mining_tank_v); //获取V字
void StrenthenColor(cv::Mat &src, cv::Mat &dst, rm::ArmorColor color); //颜色增强
double triangleArea3D(const cv::Point3d& A, const cv::Point3d& B, const cv::Point3d& C); //计算三角形面积
#endif