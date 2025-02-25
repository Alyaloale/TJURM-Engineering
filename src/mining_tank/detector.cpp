#include "mining_tank/detector.h"
#include <filesystem>
#include <opencv2/core/utils/filesystem.hpp>
#include "data_manager/control/control.h"

static int binary_ratio;
cv::Mat image_in;
cv::Mat show_contour;
cv::Mat show_triangle;
using namespace rm;
namespace fs = std::filesystem;

void Control::detect_start()
{
    while(true)
    {
        //读取图片
        if(!Data::debug)
        {
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
            Data::image_in = frame->image->clone();
            detect(Data::image_in);
            //send_single(target_yaw, target_pitch, fire, Data::target_id);
            cv::waitKey(1);

        }
        else if(Data::debug==1)
        {
            std::vector<cv::String> image_list;
            cv::utils::fs::glob(Data::read_path, "*.jpg", image_list);
            for(auto image : image_list)
            {
                cv::Mat src = cv::imread(image);
                if(src.empty()){
                    std::cout<<"image is empty"<<std::endl;
                    return;
                }
                std::cout<<image<<std::endl;
                detect(src);
                cv::waitKey(0);
            }
        }
        else if(Data::debug==2)
        {
            cv::Mat src = cv::imread("/home/tjurm/Code/TJURM-Engineering/image/15.jpg");
            if(src.empty()){
            std::cout<<"image is empty"<<std::endl;
            return;
            }
            detect(src);
            cv::waitKey(0);
        }
    }
}
void detect(cv::Mat &src)
{
    auto param = Param::get_instance();
    image_in = src.clone();

    MiningTankFour mining_tank_four;
    MiningTankV mining_tank_v;


//灰度化
    cv::Mat gray_image;
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    if(Data::self_color == rm::ARMOR_COLOR_RED)gray_image = channels[2];
    else
        if(Data::self_color == rm::ARMOR_COLOR_BLUE)gray_image = channels[0]-channels[2];
    else{
        std::cout<<"self color is not red or blue"<<std::endl;
        return;
    }
    //StrenthenColor(src,gray_image,Data::mining_tank_color);
    if(Data::show_image_flag&&Data::show_binary_image_flag){
        cv::imshow("gray_image", gray_image);
    }

//二值化
    cv::Mat binary_image;
    // if (Data::mining_tank_color == rm::ARMOR_COLOR_RED){
    //     binary_ratio=(*param)["Point"]["Threshold"]["RatioRed"];
    // } else{
    //     binary_ratio=(*param)["Point"]["Threshold"]["RatioBlue"];
    // }
    // int threshold_from_hist = rm::getThresholdFromHist(image_in,8,binary_ratio);
    // threshold_from_hist = std::clamp(threshold_from_hist, 10, 100);
    // rm::getBinary(gray_image, binary_image,threshold_from_hist, rm::BINARY_METHOD_DIRECT_THRESHOLD);
    double threshold = cv::threshold(gray_image, binary_image, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    //std::cout<<"threshold is "<<threshold<<std::endl;


//轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    show_contour = cv::Mat::zeros(image_in.size(), CV_8UC3);
    cv::findContours(binary_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //筛选获取四点轮廓
    std::vector<std::vector<cv::Point>> mining_tank_contours_four = MiningTankCountourFourSift(contours,show_contour);
    //筛选获取V字轮廓
    std::vector<std::vector<cv::Point>> mining_tank_contours_v = MiningTankCountourVSift(contours,show_contour);
    //GetMaxTwoLine(mining_tank_contours_four,binary_image);
    if(Data::show_image_flag&&Data::show_contour_flag){
        cv::imshow("show_contour", show_contour);
    }


    show_triangle = cv::Mat::zeros(image_in.size(), CV_8UC3);

    //获取四点
    mining_tank_four = GetMiningTankFour(mining_tank_contours_four,binary_image,show_triangle);
    std::cout<<"four point contour num is "<<mining_tank_contours_four.size()<<std::endl;

    //获取V字
    mining_tank_v = GetMiningTankV(mining_tank_contours_v,binary_image,show_triangle);
    std::cout<<"V contour num is "<<mining_tank_contours_v.size()<<std::endl;
    

    if(Data::show_image_flag&&Data::show_binary_image_flag){
        cv::imshow("binary_image", binary_image);
    }
    if(Data::show_image_flag&&Data::show_triangle_flag){
        cv::imshow("show_triangle", show_triangle);
    }
    if(Data::show_image_flag){
        //四点
        for(int i=0;i<mining_tank_four.point.size();i++){
            cv::line(image_in,mining_tank_four.point[i][0],mining_tank_four.point[(i+1)%mining_tank_four.point.size()][0],cv::Scalar(0,255,0),1);
        }
        if(mining_tank_contours_four.size())cv::circle(image_in,mining_tank_four.point[0][0],2,cv::Scalar(255,0,0),2);

        //V字
        if(mining_tank_contours_v.size()>=2){
            cv::line(image_in,mining_tank_v.point[0],mining_tank_v.point[1],cv::Scalar(0,255,0),1);
            cv::line(image_in,mining_tank_v.point[0],mining_tank_v.point[2],cv::Scalar(0,255,0),1);
            cv::circle(image_in,mining_tank_v.point[0],2,cv::Scalar(255,0,0),2);
        }
        cv::imshow("image", image_in);
    }
}


std::vector<std::pair<cv::Vec4d,cv::Vec4d>> GetMaxTwoLine(std::vector<std::vector<cv::Point>> contours,cv::Mat contour_image)
{
    std::vector<std::pair<cv::Vec4d,cv::Vec4d>> max_two_line;
    cv::Vec4d first_max_line, second_max_line;
    double max_length = 0;
    double second_max_length = 0;
    //霍夫变换获取两条最长线段
    for (auto &contour : contours) {
        //roi获取
        cv::RotatedRect rect = cv::minAreaRect(contour);
        int x_min=10000, x_max=0, y_min=10000, y_max=0, width=0, height=0;
        for(int i=0;i<contour.size();i++){
            x_min = std::min(x_min,contour[i].x);
            x_max = std::max(x_max,contour[i].x);
            y_min = std::min(y_min,contour[i].y);
            y_max = std::max(y_max,contour[i].y);
        }
        cv::Point2i roi = cv::Point2i(x_min,y_min);
        width = x_max-x_min;
        height = y_max-y_min;
        width = std::min(width+20,contour_image.cols-roi.x);
        height = std::min(height+20,contour_image.rows-roi.y);
        roi.x = std::max(roi.x-10,0);
        roi.y = std::max(roi.y-10,0);
        cv::Mat roi_image = contour_image(cv::Rect(roi.x,roi.y,width,height));
        // cv::imshow("roi_image",roi_image);
        // cv::waitKey(0);
        std::vector<cv::Vec4d> lines;
        cv::HoughLinesP(roi_image, lines, 1, CV_PI / 180, 50, 30, 10);
        //根据长度由大到小排序
        std::sort(lines.begin(), lines.end(), [](cv::Vec4d a, cv::Vec4d b) {
            return cv::norm(cv::Point2f(a[0], a[1]) - cv::Point2f(a[2], a[3])) > cv::norm(cv::Point2f(b[0], b[1]) - cv::Point2f(b[2], b[3]));
        });
        if (lines.size() >= 2) {
            first_max_line = lines[0];
            second_max_line = lines[1];
            //roi修正
            first_max_line[0] += roi.x;
            first_max_line[1] += roi.y;
            first_max_line[2] += roi.x;
            first_max_line[3] += roi.y;
            second_max_line[0] += roi.x;
            second_max_line[1] += roi.y;
            second_max_line[2] += roi.x;
            second_max_line[3] += roi.y;
            max_two_line.push_back(std::make_pair(first_max_line, second_max_line));
        }
        std::cout<<lines.size()<<std::endl;
    }

    return max_two_line;
}