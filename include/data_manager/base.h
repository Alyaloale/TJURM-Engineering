#ifndef DATA_MANAGER_BASE_H
#define DATA_MANAGER_BASE_H
#include <opencv2/opencv.hpp>
#include <openrm.h>
#include <opencv2/aruco.hpp>


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
    extern bool serial_flag;
    extern int send_wait_time_ms;
    extern std::vector<std::pair<std::vector<cv::Point3f>,std::vector<cv::Point2f>>> points_3D_2D;
    extern std::vector<std::vector<cv::Point3f>> points_3D;
    extern cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName;
    extern cv::Mat image_in;
    extern std::vector<int> markerIds;
    extern std::vector<std::vector<cv::Point2f>> markerCorners;
}


bool init_camera();//初始化相机
void init_serial();//初始化串口
void init_debug();//初始化调试参数
#endif // BASE_H