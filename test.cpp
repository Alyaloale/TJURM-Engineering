#include <librealsense2/rs.hpp> // RealSense 头文件
#include <opencv2/opencv.hpp>   // OpenCV 头文件
#include <iostream>

int main() {
    // 1. 初始化 RealSense 管道
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30); // 启用深度流
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30); // 启用 RGB 流
    pipe.start(cfg);

    // 2. 创建对齐对象（将深度帧对齐到 RGB 帧）
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // 3. 获取相机内参和畸变参数
    rs2::pipeline_profile profile = pipe.get_active_profile();
    rs2::video_stream_profile color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = color_profile.get_intrinsics(); // RGB 相机内参

    // 4. 主循环
    while (true) {
        // 等待下一帧数据
        rs2::frameset frames = pipe.wait_for_frames();

        // 对齐深度帧到 RGB 帧
        frames = align_to_color.process(frames);

        // 获取对齐后的深度帧和 RGB 帧
        rs2::depth_frame aligned_depth_frame = frames.get_depth_frame();
        rs2::video_frame color_frame = frames.get_color_frame();

        // 将 RGB 帧转换为 OpenCV 格式
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // 显示 RGB 图像
        cv::imshow("Color Frame", color_image);

        // 获取鼠标点击点的坐标
        if (cv::waitKey(1) == 27) break; // 按下 ESC 键退出
        if (cv::getWindowProperty("Color Frame", cv::WND_PROP_AUTOSIZE) != -1) {
            cv::setMouseCallback("Color Frame", [](int event, int x, int y, int flags, void* userdata) {
                if (event == cv::EVENT_LBUTTONDOWN) {
                    // 5. 获取鼠标点击点的深度值
                    rs2::depth_frame* aligned_depth_frame = (rs2::depth_frame*)userdata;

                    // 考虑相机畸变，校正像素坐标
                    float pixel[2] = { (float)x, (float)y };
                    float point[2];
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, 1.0f);

                    // 获取深度值
                    float depth = aligned_depth_frame->get_distance(x, y);
                    std::cout << "Pixel (" << x << ", " << y << ") Depth: " << depth << " meters" << std::endl;
                }
            }, &aligned_depth_frame);
        }
    }

    // 停止管道
    pipe.stop();
    return 0;
}