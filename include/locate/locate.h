#ifndef LOCATE_LOCATE_H
#define LOCATE_LOCATE_H
#include "data_manager/param.h"
#include "data_manager/base.h"
#include <Eigen/Dense>
#include "locate/tools.h"


std::vector<cv::Point2d> reprojectPointsToPixel(const std::vector<cv::Point3d>& srcPoints, const Eigen::Matrix4d& T, 
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);//重投影

Eigen::Matrix4d computeTransformMatrix(const std::vector<Eigen::Vector3d>& world_points,
        const std::vector<Eigen::Vector3d>& camera_points);//求解变换矩阵的函数

cv::Point3d RealSensePixelToCamera(
    const cv::Point2d& pixel_coord,
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs
);//获取去畸变后的深度值和相机坐标系下的点

cv::Point3f PixelToCameraWithoutDbscan(
    const cv::Point2f& pixel_coord,
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    bool accept_invalid_depth
);//获取深度值和相机坐标系下的点

double calculateFlatness(const std::vector<cv::Point3d>& points);//计算四点平面度

void get_image_rgbdepth();//获取深度图像

// void point_transform(cv::Point3d &point,
//     const Eigen::Matrix3d& R,
//     const Eigen::Vector3d& T);//将相机坐标系下的点进行坐标变换

bool decomposeExtrinsicMatrix(const cv::Mat& rt_rgb_to_depth,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t);//将OpenCV的外参矩阵分解为Eigen的R和t

std::vector<cv::Point3d> Cameratoimage(
    const std::vector<cv::Point3d>& camera_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs );//将相机坐标系下的点投影到图像平面
void hybridMedianBilateralFilter(const cv::Mat& depth, cv::Mat& output,
    int medianSize, 
    int bilateralSize,
    float sigmaColor,
    float sigmaSpace);//混合中值滤波器
#endif