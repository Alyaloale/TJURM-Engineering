#include "locate/locate.h"
#include <iostream>
#include <opencv2/opencv.hpp>

// 将像素坐标和深度信息转换为相机坐标系下的坐标，并进行去畸变处理
cv::Point3d pixelToCamera(const cv::Point2d& pixel_coord) {
    // 提取相机内参
    double fx = Data::realsense_camera.intrinsic_matrix.at<double>(0, 0);
    double fy = Data::realsense_camera.intrinsic_matrix.at<double>(1, 1);
    double cx = Data::realsense_camera.intrinsic_matrix.at<double>(0, 2);
    double cy = Data::realsense_camera.intrinsic_matrix.at<double>(1, 2);

    // 像素坐标转换为归一化图像坐标
    double x = (pixel_coord.x - cx) / fx;
    double y = (pixel_coord.y - cy) / fy;

    // 创建一个包含归一化图像坐标的向量
    std::vector<cv::Point2f> distorted_points;
    distorted_points.push_back(cv::Point2f(x, y));

    // 进行去畸变处理
    std::vector<cv::Point2f> undistorted_points;
    cv::undistortPoints(distorted_points, undistorted_points, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs);

    // 提取去畸变后的归一化图像坐标
    double x_undistorted = undistorted_points[0].x;
    double y_undistorted = undistorted_points[0].y;
    double depth = Data::image_in_RealSense_depth.at<u_short>(pixel_coord);
    // 转换到相机坐标系
    double X_c = x * depth;
    double Y_c = y * depth;
    double Z_c = depth;

    return cv::Point3d(X_c, Y_c, Z_c);
}

std::pair<cv::Vec3d, cv::Point3d> fitPlane(const std::vector<cv::Point3d>& points) {
    cv::Mat pointsMat(points.size(), 3, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
        pointsMat.at<double>(i, 0) = points[i].x;
        pointsMat.at<double>(i, 1) = points[i].y;
        pointsMat.at<double>(i, 2) = points[i].z;
    }

    cv::Mat centered = pointsMat - cv::mean(pointsMat);
    cv::Mat cov;
    // 手动计算均值向量
    cv::Scalar mean = cv::mean(pointsMat);
    cv::Mat meanVector(1, pointsMat.cols, pointsMat.type());
    for (int i = 0; i < pointsMat.cols; ++i) {
        meanVector.at<double>(0, i) = mean[i];
    }
    cv::calcCovarMatrix(centered, cov, meanVector, cv::COVAR_NORMAL | cv::COVAR_ROWS);
    cv::Mat eigenVectors, eigenValues;
    // 确保矩阵不为空指针
    if (eigenVectors.empty()) {
        eigenVectors.create(cov.rows, cov.cols, cov.type());
    }
    if (eigenValues.empty()) {
        eigenValues.create(cov.rows, 1, cov.type());
    }

    cv::eigen(cov, eigenValues, eigenVectors);

    cv::Vec3d normal(eigenVectors.at<double>(2, 0), eigenVectors.at<double>(2, 1), eigenVectors.at<double>(2, 2));
    cv::Scalar meanValue = cv::mean(pointsMat);
    cv::Point3d centroid(meanValue[0], meanValue[1], meanValue[2]);

    return std::make_pair(normal, centroid);
}



// 计算两个向量之间的旋转矩阵
Eigen::Matrix3d rotationBetweenVectors(const cv::Vec3d& a, const cv::Vec3d& b) {
    Eigen::Vector3d vecA(a[0], a[1], a[2]);
    Eigen::Vector3d vecB(b[0], b[1], b[2]);

    Eigen::Vector3d v = vecA.cross(vecB);
    double c = vecA.dot(vecB);
    double s = v.norm();
    Eigen::Matrix3d skewV;
    skewV << 0, -v(2), v(1),
             v(2), 0, -v(0),
            -v(1), v(0), 0;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + skewV + skewV * skewV * ((1 - c) / (s * s));
    return R;
}

// 计算变换矩阵
Eigen::Matrix4d computeTransformation(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints) {
    // 拟合源点和目标点的平面
    auto [srcNormal, srcCentroid] = fitPlane(srcPoints);
    auto [dstNormal, dstCentroid] = fitPlane(dstPoints);

    // 计算旋转矩阵
    Eigen::Matrix3d R = rotationBetweenVectors(srcNormal, dstNormal);

    // 选择一个参考点计算平移向量
    cv::Point3d srcRefPoint = srcPoints[0];
    cv::Point3d dstRefPoint = dstPoints[0];
    Eigen::Vector3d srcRefVec(srcRefPoint.x, srcRefPoint.y, srcRefPoint.z);
    Eigen::Vector3d dstRefVec(dstRefPoint.x, dstRefPoint.y, dstRefPoint.z);
    Eigen::Vector3d t = dstRefVec - R * srcRefVec;

    // 组合变换矩阵
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = R;
    transformationMatrix.block<3, 1>(0, 3) = t;

    return transformationMatrix;
}