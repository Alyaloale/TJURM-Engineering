#include "locate/locate.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "cuda/cudatool.h"

cv::Point3d RealSensePixelToCamera(
    const cv::Point2d& pixel_coord,
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs
) {
    // 1. 输入验证
    if (depth_image.empty() || camera_matrix.empty() || dist_coeffs.empty()) {
        throw std::invalid_argument("Empty input matrix!");
    }
    if (pixel_coord.x < 0 || pixel_coord.x >= depth_image.cols ||
        pixel_coord.y < 0 || pixel_coord.y >= depth_image.rows) {
        return cv::Point3d(-1, -1, -1);
    }


    // 3. 读取深度值（假设深度图为ushort类型，单位毫米）
    std::vector<pointWithDepth> dataset;
    for(int i=-3;i<3;i++)
    {
        for(int j=-3;j<3;j++)
        {
            pointWithDepth point;
            point.pos.x = pixel_coord.x+i;
            point.pos.y = pixel_coord.y+j;
            point.depth = depth_image.at<ushort>(point.pos.y,point.pos.x);
            if(point.depth)
            {
                dataset.push_back(point);
                //depth = std::min(depth,point.depth);
            }
        }
    }
    double depth_mm = getDepthWithDbscan(dataset, 5, 1);
    if (depth_mm <= 0 || depth_mm > 10000) { // 假设有效深度0.1m~10m
        return cv::Point3d(-1, -1, -1);
    }
    double depth = depth_mm ; // 转换为米

    // 4. 计算相机坐标系坐标
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    double X_c = (pixel_coord.x - cx) * depth / fx;
    double Y_c = (pixel_coord.y - cy) * depth / fy;
    double Z_c = depth;

    return cv::Point3d(X_c, Y_c, Z_c);
}


cv::Point3d PixelToCameraWithoutDbscan(
    const cv::Point2d& pixel_coord,
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix
) {
    // 1. 输入验证
    if (depth_image.empty() || camera_matrix.empty() ) {
        throw std::invalid_argument("Empty input matrix!");
    }
    if (pixel_coord.x < 0 || pixel_coord.x >= depth_image.cols ||
        pixel_coord.y < 0 || pixel_coord.y >= depth_image.rows) {
        return cv::Point3d(-1, -1, -1);
    }


    // 3. 读取深度值（假设深度图为ushort类型，单位毫米）
    double depth_mm = depth_image.at<ushort>(pixel_coord.y, pixel_coord.x);

    // 4. 计算相机坐标系坐标
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    double X_c = (pixel_coord.x - cx) * depth_mm / fx;
    double Y_c = (pixel_coord.y - cy) * depth_mm / fy;
    double Z_c = depth_mm;

    return cv::Point3d(X_c, Y_c, Z_c);
}


//重投影
std::vector<cv::Point2d> reprojectPointsToPixel(const std::vector<cv::Point3d>& srcPoints, const Eigen::Matrix4d& T, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    std::vector<cv::Point2d> pixelPoints;

    // 从变换矩阵中提取旋转矩阵
    cv::Mat rotationMatrix(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotationMatrix.at<double>(i, j) = T(i, j);
        }
    }

    // 从变换矩阵中提取平移向量
    cv::Mat translationVector(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        translationVector.at<double>(i, 0) = T(i, 3);
    }

    // 将旋转矩阵转换为旋转向量
    cv::Mat rotationVector;
    cv::Rodrigues(rotationMatrix, rotationVector);

    // 调用 projectPoints 函数进行投影
    cv::projectPoints(srcPoints, rotationVector, translationVector, cameraMatrix, distCoeffs, pixelPoints);

    return pixelPoints;
}

// 求解变换矩阵的函数
Eigen::Matrix4d computeTransformMatrix(const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Vector3d>& camera_points) {
    // 参数校验
    if (world_points.size() != 4 || camera_points.size() != 4) {
    throw std::invalid_argument("点集数量必须为4");
    }

    // 1. 计算质心
    Eigen::Vector3d world_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d camera_centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < 4; ++i) {
    world_centroid += world_points[i];
    camera_centroid += camera_points[i];
    }
    world_centroid /= 4;
    camera_centroid /= 4;

    // 2. 去质心坐标
    std::vector<Eigen::Vector3d> world_centered(4), camera_centered(4);
    for (int i = 0; i < 4; ++i) {
    world_centered[i] = world_points[i] - world_centroid;
    camera_centered[i] = camera_points[i] - camera_centroid;
    }

    // 3. 构造H矩阵
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 4; ++i) {
    H += world_centered[i] * camera_centered[i].transpose();
    }

    // 4. SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    //处理反射情况
    if (R.determinant() < 0) {
    Eigen::Matrix3d V_corrected = svd.matrixV();
    V_corrected.col(2) *= -1;
    R = V_corrected * svd.matrixU().transpose();
    }

    // 5. 计算平移向量
    Eigen::Vector3d t = camera_centroid - R * world_centroid;

    // 6. 组合变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    return T;
}


// 计算四点平面度
double calculateFlatness(const std::vector<cv::Point3d>& points) {
    if (points.size() != 4) {
        throw std::invalid_argument("必须输入四个点");
    }

    // 构造矩阵 A 和向量 B
    Eigen::MatrixXd A(4, 3);
    Eigen::VectorXd B(4);

    for (int i = 0; i < 4; ++i) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = 1.0;
        B(i) = points[i].z;
    }

    // 最小二乘法拟合平面
    Eigen::Vector3d coeffs = A.colPivHouseholderQr().solve(B);
    double a = coeffs(0);
    double b = coeffs(1);
    double c = -1.0;
    double d = coeffs(2);

    // 计算点到平面的距离
    double max_distance = 0.0;
    for (const auto& point : points) {
        double distance = std::abs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a * a + b * b + c * c);
        if (distance > max_distance) {
            max_distance = distance;
        }
    }

    return max_distance;
}

void get_image_rgbdepth()
{
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    // 将Mat转换为Eigen
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R(i,j) = Data::realsense_camera.r_rgb_to_depth.at<double>(i,j);
        }
        T(i) = Data::realsense_camera.t_rgb_to_depth.at<double>(0,i);
    }
    //转换成float
    Eigen::Matrix3f Rf = R.cast<float>();
    Eigen::Vector3f Tf = T.cast<float>();


    std::vector<float>* camera_point_in_world = new std::vector<float>;
    double fx = Data::realsense_camera.intrinsic_matrix.at<double>(0,0);
    double fy = Data::realsense_camera.intrinsic_matrix.at<double>(1,1);
    double cx = Data::realsense_camera.intrinsic_matrix.at<double>(0,2);
    double cy = Data::realsense_camera.intrinsic_matrix.at<double>(1,2);
    double scale = 1.0;
    //深度图转点云
    cv::Mat rs_depth = Data::image_in_RealSense_depth.clone();
    rs_depth.convertTo(rs_depth, CV_32F);
    depth_to_pointcloud(camera_point_in_world, rs_depth,
    fx, fy, cx, cy, scale);

    //坐标系变换CUDA加速
    point_transform(camera_point_in_world,Rf,Tf);

    //获取内参
    double fx_DaHeng = Data::camera[1]->intrinsic_matrix.at<double>(0,0);
    double fy_DaHeng = Data::camera[1]->intrinsic_matrix.at<double>(1,1);
    double cx_DaHeng = Data::camera[1]->intrinsic_matrix.at<double>(0,2);
    double cy_DaHeng = Data::camera[1]->intrinsic_matrix.at<double>(1,2);
    //内参转换成float
    float fx_DaHengf = fx_DaHeng;
    float fy_DaHengf = fy_DaHeng;
    float cx_DaHengf = cx_DaHeng;
    float cy_DaHengf = cy_DaHeng;

    int width = Data::image_in_RealSense_depth.cols;
    int height = Data::image_in_RealSense_depth.rows;

    //分配内存
    std::vector<float>* depth = new std::vector<float>;

    //将点云投影到深度图
    project_pointcloud_to_depth(camera_point_in_world, depth,fx_DaHengf,fy_DaHengf,cx_DaHengf,cy_DaHengf,width,height);

    Data::image_in_DaHeng_depth = cv::Mat::zeros(height,width,CV_16UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            Data::image_in_DaHeng_depth.at<ushort>(i,j) = static_cast<ushort>(depth->at(i*width+j));
        }
    }
    depth->clear();
    delete depth;
    camera_point_in_world->clear();
    delete camera_point_in_world;

}


// 将OpenCV的外参矩阵分解为Eigen的R和t
bool decomposeExtrinsicMatrix(const cv::Mat& rt_rgb_to_depth,
                              Eigen::Matrix3d& R,
                              Eigen::Vector3d& t) {
    // 检查输入矩阵尺寸
    if (rt_rgb_to_depth.rows != 3 || rt_rgb_to_depth.cols != 4) {
        std::cerr << "Error: Extrinsic matrix must be 3x4 [R|t] format!" << std::endl;
        return false;
    }
    if (rt_rgb_to_depth.type() != CV_64F) {
        std::cerr << "Warning: Converting matrix to double precision." << std::endl;
        cv::Mat double_mat;
        rt_rgb_to_depth.convertTo(double_mat, CV_64F);
        return decomposeExtrinsicMatrix(double_mat, R, t);
    }

    // 提取旋转矩阵部分 (OpenCV为行优先，Eigen默认列优先)
    cv::Mat cv_R = rt_rgb_to_depth(cv::Rect(0, 0, 3, 3)).clone();
    
    // 转换为Eigen矩阵（需转置，因OpenCV为行优先）
    R = Eigen::Map<Eigen::Matrix3d>(cv_R.ptr<double>()).transpose();

    // 提取平移向量部分
    cv::Mat cv_t = rt_rgb_to_depth.col(3).clone();
    t = Eigen::Map<Eigen::Vector3d>(cv_t.ptr<double>());

    // 验证旋转矩阵的有效性
    const double det = R.determinant();
    if (std::abs(det - 1.0) > 1e-6) {
        std::cerr << "Warning: Rotation matrix determinant is " << det
                  << ", performing orthogonalization." << std::endl;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();  // 强制正交化
    }

    return true;
}


