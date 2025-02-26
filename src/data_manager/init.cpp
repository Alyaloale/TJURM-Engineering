#include"data_manager/base.h"
#include"data_manager/param.h"
#include"data_manager/control/control.h"
#include <thread>
using namespace rm;


bool init_camera(){
    auto param = Param::get_instance();


    // 获取相机参数矩阵json
    nlohmann::json camlens;
    std::string camlen_path = (*param)["Camera"]["CamLensDir"];
    try {
        std::ifstream camlens_json(camlen_path);
        camlens_json >> camlens;
        camlens_json.close();
    } catch (std::exception& e) {
        std::string err_str = "Failed to load CamLens json: " + std::string(e.what());
        rm::message(err_str, rm::MSG_ERROR);
        return false;
    }


    // 获取相机数量
    int camera_num;
    bool flag_camera = rm::getDaHengCameraNum(camera_num);
    Data::camera.clear();
    Data::camera.resize(camera_num + 1, nullptr);
    if(!flag_camera) {
        rm::message("Failed to get camera number", rm::MSG_ERROR);
        return false;
    }
        rm::message("get camera number "+ std::to_string(camera_num), rm::MSG_NOTE);
        double exp = (*param)["Camera"]["Base"]["ExposureTime"];
        double gain = (*param)["Camera"]["Base"]["Gain"];
        double fps = (*param)["Camera"]["Base"]["FrameRate"];
        int roi_width = (*param)["Camera"]["Base"]["ROIWidth"];
        int roi_height = (*param)["Camera"]["Base"]["ROIHeight"];
        std::string camera_type = (*param)["Camera"]["Base"]["CameraType"];
        std::string lens_type = (*param)["Camera"]["Base"]["LensType"];
        Data::camera[1] = new rm::Camera();
        flag_camera = rm::openDaHeng(
            Data::camera[1], 1, nullptr, nullptr, nullptr, false);

        if(!flag_camera) {
            rm::message("Failed to open camera", rm::MSG_ERROR);
            return false;
        }

        flag_camera = setDaHengArgs(Data::camera[1], exp, gain, fps, rm::TRIGGER_MODE_AUTO);
        if(!flag_camera) {
            rm::message("Failed to set camera args", rm::MSG_ERROR);
            return false;
        }
        Param::from_json(camlens[camera_type][lens_type]["Intrinsic"], Data::camera[1]->intrinsic_matrix);
        Param::from_json(camlens[camera_type][lens_type]["Distortion"], Data::camera[1]->distortion_coeffs);
        return true;
}

void init_serial() {
    int status;
    std::vector<std::string> port_list;
    auto control = Control::get_instance();

    while(true) {

        status = (int)rm::getSerialPortList(port_list, rm::SERIAL_TYPE_TTY_USB);

        if (status != 0 || port_list.empty()) {
            rm::message("Control port list failed", rm::MSG_ERROR);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            port_list.clear();
            continue;
        }

        control->port_name_ = port_list[0];
        status = (int)rm::openSerialPort(control->file_descriptor_, control->port_name_);
        if (status != 0) {
            rm::message("Control port open failed", rm::MSG_ERROR);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            port_list.clear();
            continue;
        }
        if(status == 0) {
            break;
        }
    }
}

void init_debug() {
    auto param = Param::get_instance();


    Data::serial_flag = (*param)["Debug"]["Control"]["Serial"];
    Data::debug = (*param)["Debug"]["Debug"];
    Data::mining_tank_color = rm::ARMOR_COLOR_RED;
    Data::self_color = rm::ARMOR_COLOR_RED;
    Data::read_path = (*param)["Path"]["ImagePath"];
    Data::show_image_flag = (*param)["Debug"]["ShowImage"];
    Data::show_binary_image_flag = (*param)["Debug"]["BinaryImage"];
    Data::show_contour_flag = (*param)["Debug"]["ShowContour"];
    Data::show_triangle_flag = (*param)["Debug"]["ShowTriangle"];
    Data::send_wait_time_ms = (*param)["Debug"]["Control"]["SendWaitTime"];
    Data::serial_flag = (*param)["Debug"]["Control"]["Serial"];
    Data::show_aruco = (*param)["Debug"]["ShowAruco"];
    Data::timeout_flag = (*param)["Debug"]["TimeOut"];

    std::vector<cv::Point3f> Point1;
    cv::Point3f point1,point2,point3;
    point1.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["point3"]["z"];
    Point1.push_back(point1);
    Point1.push_back(point2);
    Point1.push_back(point3);

    std::vector<cv::Point3f> Point2;
    point1.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["point3"]["z"];
    Point2.push_back(point1);
    Point2.push_back(point2);
    Point2.push_back(point3);

    std::vector<cv::Point3f> Point3;
    point1.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["point3"]["z"];
    Point3.push_back(point1);
    Point3.push_back(point2);
    Point3.push_back(point3);

    std::vector<cv::Point3f> Point4;
    point1.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["point3"]["z"];
    Point4.push_back(point1);
    Point4.push_back(point2);
    Point4.push_back(point3);
    
    std::vector<cv::Point3f> Point5;
    point1.x = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point3"]["z"];
    Point5.push_back(point1);
    Point5.push_back(point2);
    Point5.push_back(point3);

    std::vector<cv::Point3f> Point6;
    point1.x = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point3"]["z"];
    Point6.push_back(point1);
    Point6.push_back(point2);
    Point6.push_back(point3);


    Data::points_3D.push_back(Point1);
    Data::points_3D.push_back(Point2);
    Data::points_3D.push_back(Point3);
    Data::points_3D.push_back(Point4);
    Data::points_3D.push_back(Point5);
    Data::points_3D.push_back(Point6);
}