#ifndef MINING_TANK_DETECTOR_H_
#define MINING_TANK_DETECTOR_H_
#include "data_manager/param.h"
#include "data_manager/base.h"
#include "mining_tank/get_Four.h"
#include "mining_tank/get_V.h"



void detect_start();
void detect(cv::Mat &src);
void check(std::vector<std::vector<cv::Point2f>>* mining_tank_four, std::vector<cv::Point2f>* mining_tank_v, std::pair<int,int> &flag);
bool locate(std::vector<std::vector<cv::Point2f>>* mining_tank_four, std::vector<cv::Point2f>* mining_tank_v, std::pair<int,int> flag, bool IsFourOrTwelve);
void orderPointsThreeAnticlockwise(std::vector<cv::Point2f>& points);//将三点按逆时针排序

#endif