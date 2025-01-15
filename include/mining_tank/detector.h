#ifndef MINING_TANK_DETECTOR_H_
#define MINING_TANK_DETECTOR_H_
#include "data_manager/param.h"
#include "data_manager/base.h"
#include "mining_tank/get_Four.h"
#include "mining_tank/get_V.h"



void detect_start();
void detect(cv::Mat &src);
std::vector<std::pair<cv::Vec4d,cv::Vec4d>> GetMaxTwoLine(std::vector<std::vector<cv::Point2f>> contours,cv::Mat contour_image); //霍夫变换获取两条最长线段

#endif