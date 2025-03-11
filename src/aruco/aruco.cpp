#include "aruco/aruco_locate.h"
#include "data_manager/control/control.h"
#include "locate/locate.h"


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
    int count = 0;
    while(true)
    {
        cv::Mat aruco_image;
        get_image_RealSense();
        //get_image_DaHeng();
        // cv::imshow("RealSense", Data::image_in_RealSense_depth);
        // cv::imshow("RealSense_color", Data::image_in_RealSense_color);
        if(!Data::image_in_RealSense_color.empty())
        {
            bool IsdetectAruco;
            Data::markerIds.clear();
            Data::markerCorners.clear();
            IsdetectAruco = detectArucoMarkers(Data::image_in_RealSense_color.clone(), Data::markerIds, Data::markerCorners, true, &aruco_image);
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
                //按q保存图片，序号命名
                // if(cv::waitKey(1) == 'q')
                // {
                //     cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/biaoding/"+std::to_string(count++)+".jpg", Data::image_in_RealSense_color);
                // }
                for(int i=0;i<Data::markerIds.size();i++)
                {
                    std::cout<<"id: "<<Data::markerIds[i]<<std::endl;
                    for(int j=0;j<Data::markerCorners[i].size();j++)
                    {
                        std::cout<<"corner: "<<Data::markerCorners[i][j]<<std::endl;
                    }
                }
            }
            if(IsdetectAruco)
            {
                getArucotocamera(Data::markerIds, Data::markerCorners);
            }
        }
    }
}
void getArucotocamera(std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f>>& markerCorners)
{
    std::vector< std::pair<std::vector<cv::Point3d>, int> > Arucos_camera;
    for(int i=0;i<markerIds.size();i++)
    {
        std::pair<std::vector<cv::Point3d>, int> Aruco_camera;
        Aruco_camera.second = markerIds[i];
        for(int j=0;j<markerCorners[i].size();j++)
        {
            cv::Point2d point;
            point.x = markerCorners[i][j].x;
            point.y = markerCorners[i][j].y;
            cv::Point3d point3d = pixelToCamera(point);
            if(point3d.x&&point3d.y&&point3d.z)Aruco_camera.first.push_back(point3d);
        }
        if(Aruco_camera.first.size()==4)Arucos_camera.push_back(Aruco_camera);
    }
    //输出距离
    // for(int i=0;i<Arucos_camera.size();i++)
    // {
    //     std::cout<<"id: "<<Arucos_camera[i].second<<std::endl;
    //     for(int j=0;j<Arucos_camera[i].first.size();j++)
    //     {
    //         std::cout<<"distance: "<<Arucos_camera[i].first[j]<<std::endl;
    //     }
    // }

    for(int i=0;i<Arucos_camera.size();i++)
    {
        Eigen::Matrix4d T;
        std::vector<cv::Point3d> srcPoints = {
            cv::Point3d(0, 0, 0),
            cv::Point3d(1, 0, 0),
            cv::Point3d(1, 1, 0),
            cv::Point3d(0, 1, 0)
        };
        T = computeTransformation(srcPoints, Arucos_camera[i].first);
        std::cout<<"T: "<<T<<std::endl;
    }
}
