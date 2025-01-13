#include"data_manager/base.h"
#include"data_manager/param.h"
using namespace rm;


void init(){
    Data::mining_tank_color = rm::ARMOR_COLOR_RED;
    if(!init_camera())
    {
        rm::message("Failed to init camera", rm::MSG_ERROR);
        exit(-1);
    }
}


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
        double fps = (*param)["Camera"]["Base"]["FPS"];
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

        flag_camera = setDaHengArgs(Data::camera[1], exp, gain, fps, 0, roi_width, roi_height);
        if(!flag_camera) {
            rm::message("Failed to set camera args", rm::MSG_ERROR);
            return false;
        }
        Param::from_json(camlens[camera_type][lens_type]["Intrinsic"], Data::camera[1]->intrinsic_matrix);
        Param::from_json(camlens[camera_type][lens_type]["Distortion"], Data::camera[1]->distortion_coeffs);
}