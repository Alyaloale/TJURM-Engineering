#include "aruco/aruco_locate.h"
#include "data_manager/control/control.h"
#include "locate/locate.h"


void aruco_cereate()
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    for(int i=0;i<4;i++)
    {
        cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
        cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/aruco/"+std::to_string(i)+".jpg", markerImage);
    }
}


//  检测 ArUco 标记的函数
bool detectArucoMarkers(const cv::Mat& inputImage,
    std::vector<int>& markerIds, std::vector<std::vector<cv::Point2f>>& markerCorners,
    bool drawMarkers ,std::vector<std::vector<cv::Point2f>>& rejectedCandidates)
{
    cv::Mat imageCopy;
    inputImage.copyTo(imageCopy);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(Data::dictionaryName);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::detectMarkers(imageCopy, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
    if (drawMarkers) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    }
    return !markerIds.empty();
}

void Control::aruco_detect() {
    auto param = Param::get_instance();
    int count = 0;
    while(true)
    {
        get_image_RealSense();
        if(!Data::image_in_RealSense_color.empty()&&Data::show_aruco)
        {
            bool IsdetectAruco;
            Data::markerIds.clear();
            Data::markerCorners.clear();
            IsdetectAruco = detectArucoMarkers(Data::image_in_RealSense_color, Data::markerIds, Data::markerCorners, true, Data::rejectedCandidates);
            if(IsdetectAruco)
            {
                //画出检测到的aruco
                cv::aruco::drawDetectedMarkers(Data::image_in_RealSense_color, Data::markerCorners, Data::markerIds);
                // for(int i=0;i<Data::markerIds.size();i++)
                // {
                //     std::cout<<"id: "<<Data::markerIds[i]<<std::endl;
                //     for(int j=0;j<Data::markerCorners[i].size();j++)
                //     {
                //         std::cout<<"corner: "<<Data::markerCorners[i][j]<<std::endl;
                //     }
                // }

                //计算相机到aruco的相机坐标系坐标和变换矩阵
                getArucotocamera(Data::markerIds, Data::markerCorners);
            }

            if(Data::show_aruco)
            {
                //cv::resize(Data::image_in_RealSense_color, Data::image_in_RealSense_color, cv::Size(3840, 2160));
                cv::imshow("Aruco", Data::image_in_RealSense_color);
                cv::imshow("depth", Data::image_in_RealSense_depth);
                //输入w保存图片
                char key = cv::waitKey(1);
                // if(key=='w')
                // {
                //     cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/ARUCO/"+std::to_string(count)+".jpg", Data::image_in_RealSense_color);
                //     cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/RGB/"+std::to_string(count)+".jpg", Data::image_in_DaHeng);
                //     count++;
                // }
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
            cv::Point3d point3d = RealSensePixelToCamera(point, Data::image_in_RealSense_depth, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs);
            //std::cout<<"x: "<<point3d.x<<" y: "<<point3d.y<<" z: "<<point3d.z<<std::endl;
            if(point3d.z > 0)Aruco_camera.first.push_back(point3d);
        }
        if(Aruco_camera.first.size()==4)Arucos_camera.push_back(Aruco_camera);
    }
    //输出距离
    for(int i=0;i<Arucos_camera.size();i++)
    {
        std::cout<<"id: "<<Arucos_camera[i].second<<std::endl;
        for(int j=0;j<Arucos_camera[i].first.size();j++)
        {
            std::cout<<"x: "<<Arucos_camera[i].first[j].x<<" y: "<<Arucos_camera[i].first[j].y<<" z: "<<Arucos_camera[i].first[j].z<<std::endl;
        }
    }

    for(int i=0;i<Arucos_camera.size();i++)
    {
        Eigen::Matrix4d T;
        std::vector<cv::Point3d> srcPoints = {
            cv::Point3d(0, 0, 0),
            cv::Point3d(70, 0, 0),
            cv::Point3d(70, 70, 0),
            cv::Point3d(0, 70, 0)
        };


        //将aruco的相机坐标系坐标转换为Eigen::Vector3d
        std::vector<Eigen::Vector3d> srcPoints_eigen;
        std::vector<Eigen::Vector3d> dstPoints_eigen;
        for(int j=0;j<srcPoints.size();j++)
        {
            Eigen::Vector3d srcPoint(srcPoints[j].x, srcPoints[j].y, srcPoints[j].z);
            srcPoints_eigen.push_back(srcPoint);
            Eigen::Vector3d dstPoint(Arucos_camera[i].first[j].x, Arucos_camera[i].first[j].y, Arucos_camera[i].first[j].z);
            dstPoints_eigen.push_back(dstPoint);
        }
        //平面度
        double flatness = calculateFlatness(Arucos_camera[i].first);
        std::cout<<"flatness: "<<flatness<<std::endl;
        //计算变换矩阵
        T = computeTransformMatrix(srcPoints_eigen, dstPoints_eigen);
        if(flatness < 10)Data::RealSenseT = T;
        //畸变矩阵设置为空
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        //重投影
        std::vector<cv::Point2d> pixelPoints = reprojectPointsToPixel(srcPoints, T, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs);

        //依次画出四个点
        for(int j=0;j<pixelPoints.size();j++)
        {
            cv::circle(Data::image_in_RealSense_color, pixelPoints[j], 5, cv::Scalar(0, 255, 0), -1);
        }
        //std::cout<<"T: "<<T<<std::endl;
    }
}
