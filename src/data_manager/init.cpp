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
        // Param::from_json(camlens["camera_type"]["lens_type"]["Intrinsic"], Data::camera[1]->intrinsic_matrix);
        // Param::from_json(camlens["camera_type"]["lens_type"]["Distortion"], Data::camera[1]->distortion_coeffs);


        //RealSense相机
        try {
            // 配置彩色图像流
            Data::realsense_camera.config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
            // 配置深度图像流
            Data::realsense_camera.config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
            // 启动管道，应用配置
            if(!Data::realsense_camera.Ispipeline_started)
            {
                Data::realsense_camera.pipeline.start(Data::realsense_camera.config);
                Data::realsense_camera.Ispipeline_started = true;
            }
            // 获取相机内参

            Param::from_json(camlens["RealSense"]["D435i"]["Intrinsic"], Data::realsense_camera.intrinsic_matrix);
            Param::from_json(camlens["RealSense"]["D435i"]["Distortion"], Data::realsense_camera.distortion_coeffs);

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
}

// 获取RealSense相机图像
void get_image_RealSense(){
    rs2::frameset frameset;
    // 等待获取一帧数据
    frameset = Data::realsense_camera.pipeline.wait_for_frames();
    //进行对齐操作
    rs2::frameset aligned_frames = Data::realsense_camera.depth_to_color.process(frameset);

    // 获取对齐后的深度帧和彩色帧
    rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
    rs2::video_frame color_frame = aligned_frames.get_color_frame();
    if (!aligned_depth_frame || !color_frame) return;

    // 将深度帧和彩色帧转换为 OpenCV 图像
    Data::image_in_RealSense_color = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    Data::image_in_RealSense_depth = cv::Mat(cv::Size(1280, 720), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
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