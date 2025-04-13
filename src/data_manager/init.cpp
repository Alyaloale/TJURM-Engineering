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
    // int camera_num;
    // bool flag_camera = rm::getDaHengCameraNum(camera_num);
    // Data::camera.clear();
    // Data::camera.resize(camera_num + 1, nullptr);
    // if(!flag_camera) {
    //     rm::message("Failed to get camera number", rm::MSG_ERROR);
    //     return false;
    // }
    //     rm::message("get camera number "+ std::to_string(camera_num), rm::MSG_NOTE);
    //     double exp = (*param)["Camera"]["Base"]["ExposureTime"];
    //     double gain = (*param)["Camera"]["Base"]["Gain"];
    //     double fps = (*param)["Camera"]["Base"]["FrameRate"];
    //     int roi_width = (*param)["Camera"]["Base"]["ROIWidth"];
    //     int roi_height = (*param)["Camera"]["Base"]["ROIHeight"];
    //     std::string camera_type = (*param)["Camera"]["Base"]["CameraType"];
    //     std::string lens_type = (*param)["Camera"]["Base"]["LensType"];
    //     Data::camera[1] = new rm::Camera();
    //     flag_camera = rm::openDaHeng(
    //         Data::camera[1], 1, nullptr, nullptr, nullptr, false);

    //     if(!flag_camera) {
    //         rm::message("Failed to open camera", rm::MSG_ERROR);
    //         return false;
    //     }

    //     flag_camera = setDaHengArgs(Data::camera[1], exp, gain, fps ,rm::TRIGGER_MODE_AUTO);
    //     if(!flag_camera) {
    //         rm::message("Failed to set camera args", rm::MSG_ERROR);
    //         return false;
    //     }
    //     Param::from_json(camlens[camera_type][lens_type]["Intrinsic"], Data::camera[1]->intrinsic_matrix);
    //     Param::from_json(camlens[camera_type][lens_type]["Distortion"], Data::camera[1]->distortion_coeffs);


    double realsense_exp = (*param)["Camera"]["RealSense"]["ExposureTime"];
    //RealSense相机
    try {
        Data::realsense_camera.config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        // 配置深度图像流
        Data::realsense_camera.config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
        // 启动管道，应用配置
        if(!Data::realsense_camera.Ispipeline_started)
        {
            Data::realsense_camera.pipeline.start(Data::realsense_camera.config);
            Data::realsense_camera.Ispipeline_started = true;
        }
        // 配置彩色图像流
        auto sensor = Data::realsense_camera.pipeline.get_active_profile().get_device().query_sensors()[1];
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0); // 关闭自动曝光
        sensor.set_option(RS2_OPTION_EXPOSURE, realsense_exp);
        // 获取相机内参

        Param::from_json(camlens["RealSense"]["D435i"]["Intrinsic"], Data::realsense_camera.intrinsic_matrix);
        Param::from_json(camlens["RealSense"]["D435i"]["Distortion"], Data::realsense_camera.distortion_coeffs);
        //std::cout<<Data::realsense_camera.intrinsic_matrix<<std::endl;
        // 获取外参
        Param::from_json(camlens["RealSense"]["D435i"]["Rotation"], Data::realsense_camera.r_rgb_to_depth);
        Param::from_json(camlens["RealSense"]["D435i"]["Translation"], Data::realsense_camera.t_rgb_to_depth);
        //获取深度标尺
        // double depth_scale = Data::realsense_camera.pipeline.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
        // std::cout << "Depth scale: " << depth_scale << std::endl;


    } catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

// 获取大恒相机图像
void get_image_DaHeng(){
    Camera* camera = Data::camera[Data::camera_index];

    std::shared_ptr<rm::Frame> frame = camera->buffer->pop();

    TimePoint frame_wait = getTime();
    while(frame == nullptr) {
        frame = camera->buffer->pop();
        double delay = getDoubleOfS(frame_wait, getTime());
        if (delay > 0.5 && Data::timeout_flag) {
            rm::message("Capture timeout", rm::MSG_ERROR);
            exit(-1);
        }
    }
    Data::image_in_DaHeng = frame->image->clone();
    cv::resize(Data::image_in_DaHeng, Data::image_in_DaHeng, cv::Size(1280, 720));
}

// 获取RealSense相机图像
void get_image_RealSense(){
    rs2::frameset frameset;
    // 等待获取一帧数据
    frameset = Data::realsense_camera.pipeline.wait_for_frames();
    //进行对齐操作
    rs2::frameset aligned_frames = Data::realsense_camera.depth_to_color.process(frameset);

    //滤波
    // rs2::decimation_filter dec_filter;   // 降采样滤波器 (降低分辨率换精度)
    // dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);  // 降采样率2x

    // rs2::spatial_filter spatial_filter;  // 空间域高斯滤波
    // spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f); 
    // spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);  // 边缘保护阈值

    // rs2::temporal_filter temp_filter;    // 时域递推滤波
    // temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    // temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);

    // rs2::hole_filling_filter hole_filter;// 空洞填充滤波
    // hole_filter.set_option(RS2_OPTION_HOLES_FILL, 2);  // 使用邻域均值填充


    // 获取对齐后的深度帧和彩色帧
    rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
    // .apply_filter(dec_filter)
    // .apply_filter(spatial_filter)
    // .apply_filter(temp_filter)
    // .apply_filter(hole_filter);
    rs2::video_frame color_frame = aligned_frames.get_color_frame();
    if (!aligned_depth_frame || !color_frame) return;

    // 将深度帧和彩色帧转换为 OpenCV 图像
    Data::image_in_RealSense_color = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    Data::image_in_RealSense_depth = cv::Mat(cv::Size(1280, 720), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);

}


void init_serial() {
    Data::shared_data = init_shared_memory();
    short color = 0;
    // while(color == 0)
    // {
    // read_shared_data(Data::shared_data, &color);
    // }
    // if(color == 1 )Data::self_color = rm::ARMOR_COLOR_RED;
    // else if(color == 2) Data::self_color = rm::ARMOR_COLOR_BLUE;
    // else {
    //     rm::message("Serial color error", rm::MSG_ERROR);
    //     exit(-1);
    // }

    rm::message("Serial color: " + std::to_string(color), rm::MSG_OK);

}

void init_debug() {
    auto param = Param::get_instance();


    Data::serial_flag = (*param)["Debug"]["Control"]["Serial"];
    Data::debug = (*param)["Debug"]["Debug"];
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


    cv::Point3d point1,point2,point3,point4;
    //获取四点世界坐标
    point1.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["x"];
    point1.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["y"];
    point1.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point1"]["z"];
    point2.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["x"];
    point2.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["y"];
    point2.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point2"]["z"];
    point3.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["x"];
    point3.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["y"];
    point3.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point3"]["z"];
    point4.x = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["x"];
    point4.y = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["y"];
    point4.z = (*param)["Point"]["MiningTank"]["Four"]["Points"]["Point4"]["z"];

    Data::points_3D.push_back(point1);
    Data::points_3D.push_back(point2);
    Data::points_3D.push_back(point3);
    Data::points_3D.push_back(point4);


    //获取V字世界坐标(右侧），V字尖朝左
    cv::Point3d point5,point6,point7;
    point5.x = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point1"]["x"];
    point5.y = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point1"]["y"];
    point5.z = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point1"]["z"];
    point6.x = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point2"]["x"];
    point6.y = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point2"]["y"];
    point6.z = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point2"]["z"];
    point7.x = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point3"]["x"];
    point7.y = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point3"]["y"];
    point7.z = (*param)["Point"]["MiningTank"]["V"]["PointRight"]["Point3"]["z"];
    Data::points_3D.push_back(point5);
    Data::points_3D.push_back(point6);
    Data::points_3D.push_back(point7);


    //获取V字世界坐标(左侧），V字尖朝右
    cv::Point3d point8,point9,point10;
    point8.x = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point1"]["x"];
    point8.y = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point1"]["y"];
    point8.z = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point1"]["z"];
    point9.x = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point2"]["x"];
    point9.y = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point2"]["y"];
    point9.z = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point2"]["z"];
    point10.x = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point3"]["x"];
    point10.y = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point3"]["y"];
    point10.z = (*param)["Point"]["MiningTank"]["V"]["PointLeft"]["Point3"]["z"];
    Data::points_3D.push_back(point8);
    Data::points_3D.push_back(point9);
    Data::points_3D.push_back(point10);
}