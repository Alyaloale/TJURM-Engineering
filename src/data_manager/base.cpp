#include"data_manager/base.h"
#include"data_manager/param.h"


rm::ArmorColor Data::self_color;
rm::ArmorColor Data::mining_tank_color;
int Data::camera_index = 1;
std::vector<rm::Camera*> Data::camera;
bool Data::timeout_flag = true;
int Data::debug = 0;
std::string Data::read_path;
bool Data::show_image_flag = true;
bool Data::show_binary_image_flag = false;
bool Data::show_contour_flag = false;
bool Data::show_triangle_flag = false;
bool Data::serial_flag = false;
int Data::send_wait_time_ms = 2000;
std::vector<std::pair<std::vector<cv::Point3f>,std::vector<cv::Point2f>> Data::points_3D_2D;
std::vector<std::vector<cv::Point3f>> Data::points_3D;