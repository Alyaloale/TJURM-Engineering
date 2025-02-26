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
    bool drawMarkers ,std::vector<std::vector<cv::Point2f>>& rejectedCandidates) {

    // 获取预定义的 ArUco 字典
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(Data::dictionaryName);

    // 创建检测参数对象
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // 检测 ArUco 标记
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    std::cout<<markerCorners.size()<<" ";
    std::cout<<rejectedCandidates.size()<<std::endl;

    return !markerIds.empty();
}

void Control::aruco_detect() {
    auto param = Param::get_instance();
    while(true)
    {
        cv::Mat aruco_image =Data::image_in.clone() ;
        if(!Data::image_in.empty())
        {
            detectArucoMarkers(Data::image_in.clone(), Data::markerIds, Data::markerCorners, true,Data::rejectedCandidates);
            //std::cout<<Data::markerIds.size()<<std::endl;
            if(Data::show_aruco)
            {
                for (size_t i = 0; i < Data::markerCorners.size(); ++i) {
                        std::vector<cv::Point> polygon;
                        for (size_t j = 0; j < Data::markerCorners[i].size(); ++j) {
                            polygon.push_back(cv::Point(Data::markerCorners[i][j].x, Data::markerCorners[i][j].y));
                        }
                        cv::polylines(aruco_image, polygon, true, cv::Scalar(0, 255, 0), 2);
                    }

                cv::imshow("aruco", aruco_image);
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
}