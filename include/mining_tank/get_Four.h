#ifndef MINING_TANK_GET_FOUR_H_
#define MINING_TANK_GET_FOUR_H_
#include "data_manager/param.h"
#include "data_manager/base.h"



void GetMiningTankFour(std::vector<std::vector<cv::Point>>* contours,cv::Mat& binary_image,cv::Mat& show_triangle, std::vector<std::vector<cv::Point2f>>* mining_tank_four); //获取四点
void MiningTankCountourFourSift(std::vector<std::vector<cv::Point>> contours,cv::Mat &show_contour, std::vector<std::vector<cv::Point>>* mining_tank_contours); //获取四点轮廓
int CountLinePixels(cv::Point p1, cv::Point p2, cv::Mat &binary_image); //计算直线上的白点数
double calculateRedness(cv::Mat& src, const std::vector<std::vector<cv::Point>>& contours);//计算红色度
double calculateBlueness(cv::Mat& src, const std::vector<std::vector<cv::Point>>& contours);//计算蓝色度
double triangleArea3D(const cv::Point3d& A, const cv::Point3d& B, const cv::Point3d& C); //计算三角形面积
void orderPointsClockwise(std::vector<std::vector<cv::Point2f>> &points, cv::Point2f &centroid); //将四点按顺时针排序
void findbestfour(std::vector<std::vector<cv::Point2f>> &fours, std::vector<std::vector<cv::Point2f>>* best_four); //获取最好的四点
#endif