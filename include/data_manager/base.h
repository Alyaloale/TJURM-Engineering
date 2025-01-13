#ifndef DATA_MANAGER_BASE_H
#define DATA_MANAGER_BASE_H
#include <opencv2/opencv.hpp>
#include <openrm.h>


struct MiningTankFour
{
    std::vector<std::vector<cv::Point2f>> point;//以三角形形式存储，顶点为第一个点，顺时针
    cv::Point2f center;
};

struct MiningTankV 
{
    std::vector<cv::Point2f> point;
};

namespace Data{
    // 颜色
    extern rm::ArmorColor self_color;
    extern rm::ArmorColor mining_tank_color;
    extern std::vector<rm::Camera*> camera;
    extern int camera_index;
    extern bool timeout_flag;
}


void init();
bool init_camera();//初始化相机
#endif // BASE_H