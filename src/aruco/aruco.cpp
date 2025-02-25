#include "aruco/aruco_locate.h"
#include "data_manager/control/control.h"


void aruco_cereate()
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    for(int i=0;i<3;i++)
    {
        cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
        cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/aruco/"+std::to_string(i)+".jpg", markerImage);
    }
}


//  检测 ArUco 标记的函数
bool detectArucoMarkers(const cv::Mat& inputImage,
    std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f>>& markerCorners,
    bool drawMarkers , cv::Mat* outputImage ) {

    // 获取预定义的 ArUco 字典
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(Data::dictionaryName);

    // 创建检测参数对象
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // 存储可能被拒绝的候选标记
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;

    // 检测 ArUco 标记
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    if (drawMarkers && outputImage != nullptr) {

        // 复制输入图像到输出图像
        inputImage.copyTo(*outputImage);
        // 绘制检测到的标记
        if (!markerIds.empty()) {

            cv::aruco::drawDetectedMarkers(*outputImage, markerCorners, markerIds);
        }
    }

    return !markerIds.empty();
}

void Control::aruco_detect() {
    auto param = Param::get_instance();
    while(true)
    {
        cv::Mat aruco_image;
        if(!Data::image_in.empty())detectArucoMarkers(Data::image_in.clone(), Data::markerIds, Data::markerCorners, true, &Data::image_in);
        if(Data::show_aruco)
        {
            cv::imshow("aruco", Data::image_in);
            cv::waitKey(1);
            for(int i=0;i<Data::markerIds.size();i++)
            {
                std::cout<<"id: "<<Data::markerIds[i]<<std::endl;
                for(int j=0;j<Data::markerCorners[i].size();j++)
                {
                    std::cout<<"corner: "<<Data::markerCorners[i][j]<<std::endl;
                }
            }
        }
    }
}