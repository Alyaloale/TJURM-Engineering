#ifndef DATA_MANAGER_BASE_H
#define DATA_MANAGER_BASE_H
#include <opencv2/opencv.hpp>
#include <openrm.h>
//#include <opencv2/aruco.hpp>
#include <librealsense2/rs.hpp>
#include"data_manager/control/control.h"

struct RealSenseCamera {
    rs2::config config;//配置摄像头
    rs2::pipeline pipeline;//启动摄像头
    rs2::pipeline_profile profile;//启动管道，应用配置
    rs2::align depth_to_color;//创建对齐对象，将深度帧对齐到彩色帧
    bool Ispipeline_started = false;
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coeffs;
    cv::Mat r_rgb_to_depth;
    cv::Mat t_rgb_to_depth;
    // 创建滤波对象
    rs2::decimation_filter dec_filter;  // 降采样滤波
    rs2::spatial_filter spatial_filter; // 空间域滤波
    rs2::temporal_filter temporal_filter; // 时间域滤波
    RealSenseCamera() : depth_to_color(rs2_stream::RS2_STREAM_COLOR) {
        
    }
};


struct MiningTankV
{
    std::vector<cv::Point2f> point;
};

struct MiningTankFour
{
    std::vector<std::vector<cv::Point2f>> point;//以三角形形式存储，顶点为第一个点，顺时针
    cv::Point2f center;
};

struct SharedData;
namespace Data{
    // 颜色
    extern rm::ArmorColor self_color;
    extern rm::ArmorColor mining_tank_color;
    extern std::vector<rm::Camera*> camera;
    extern int camera_index;
    extern bool timeout_flag;
    extern int debug;
    extern std::string read_path;
    extern bool show_image_flag;
    extern bool show_binary_image_flag;
    extern bool show_contour_flag;
    extern bool show_triangle_flag;
    extern bool show_aruco;
    extern bool IsFourOrTwelve;
    extern bool serial_flag;
    extern int send_wait_time_ms;
    extern std::vector<std::pair<std::vector<cv::Point3f>,std::vector<cv::Point2f>>> points_3D_2D;
    extern std::vector<cv::Point3d> points_3D;
    extern std::vector<std::vector<cv::Point3d>> points_3D_triangle;
    //extern cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName;
    extern cv::Mat image_in_DaHeng;
    extern cv::Mat image_in_DaHeng_depth;
    extern cv::Mat image_in_RealSense_color;
    extern cv::Mat image_in_RealSense_depth;
    extern RealSenseCamera realsense_camera;
    extern std::vector<int> markerIds;
    extern std::vector<std::vector<cv::Point2f>> markerCorners;
    extern std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    extern cv::Mat RealSenseT;
    extern double DaHengT[4][4];
    extern SharedData* shared_data;
    extern bool show_depth;
}


bool init_camera();//初始化相机
void init_serial();//初始化串口
void init_debug();//初始化调试参数
void get_image_DaHeng();//获取大恒相机图像
void get_image_RealSense();//获取RealSense相机图像
#endif // BASE_H