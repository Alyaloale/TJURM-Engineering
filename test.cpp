
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char * argv[]) try
{
    // 配置摄像头
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // 创建对齐对象，将深度帧对齐到彩色帧
    rs2::align align_to(RS2_STREAM_COLOR);

    // 启动摄像头
    pipe.start(cfg);

    while (true)
    {
        rs2::frameset frames;
        // 等待获取一帧数据
        if (!pipe.poll_for_frames(&frames))
            continue;

        // 进行对齐操作
        rs2::frameset aligned_frames = align_to.process(frames);

        // 获取对齐后的深度帧和彩色帧
        rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        rs2::video_frame color_frame = aligned_frames.get_color_frame();

        if (!aligned_depth_frame || !color_frame)
            continue;

        // 将深度帧和彩色帧转换为 OpenCV 图像
        cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // 应用颜色映射以更好地可视化深度图像
        cv::Mat depth_colormap;
        cv::convertScaleAbs(depth_image, depth_colormap, 0.03);
        cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

        // 显示对齐后的深度图像和彩色图像
        cv::Mat images;
        cv::hconcat(color_image, depth_colormap, images);
        cv::imshow("Aligned Images", images);

        // 按 'q' 键退出循环
        if (cv::waitKey(1) == 'q')
            break;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
