#ifndef LOCATE_LOCATE_H
#define LOCATE_LOCATE_H
#include "data_manager/param.h"
#include "data_manager/base.h"
#include <Eigen/Dense>


cv::Point3d pixelToCamera(const cv::Point2d& pixel_coord);//将像素坐标和深度信息转换为相机坐标系下的坐标，并进行去畸变处理
std::pair<cv::Vec3d, cv::Point3d> fitPlane(const std::vector<cv::Point3d>& points);//拟合平面，返回平面法向量和平面上一点
Eigen::Matrix4d computeTransformation(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints);//计算变换矩阵
Eigen::Matrix3d rotationBetweenVectors(const cv::Vec3d& a, const cv::Vec3d& b);//计算两个向量之间的旋转矩阵



#endif