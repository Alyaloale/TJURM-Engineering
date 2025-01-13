#include "data_manager/base.h"
#include "data_manager/param.h"
#include "mining_tank/detector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
namespace fs = std::filesystem;
using namespace rm;
int main(int argc, char** argv) {
    auto param = Param::get_instance();
    int debug = (*param)["Debug"]["Debug"];
    Data::mining_tank_color = rm::ARMOR_COLOR_RED;
    //读取图片
    if(!debug)
    {
        init();
        Camera* camera = Data::camera[Data::camera_index];
        while(true)
        {
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
            detect(*(frame->image));
            cv::imshow("src", *(frame->image));
            cv::waitKey(1);
        }
    }
    else if(debug==1)
    {
        std::string path =(*param)["Path"]["ImagePath"];
        std::vector<cv::String> image_list;
        cv::utils::fs::glob(path, "*.jpg", image_list);
        for(auto image : image_list)
        {
            cv::Mat src = cv::imread(image);
            if(src.empty()){
                std::cout<<"image is empty"<<std::endl;
                return -1;
            }
            std::cout<<image<<std::endl;
            detect(src);
            cv::waitKey(0);
        }
    }
    else if(debug==2)
    {
        cv::Mat src = cv::imread("/home/tjurm/Code/TJURM-Engineering/image/15.jpg");
        if(src.empty()){
        std::cout<<"image is empty"<<std::endl;
        return -1;
        }
        detect(src);
        cv::waitKey(0);
    }
    return 0;
}