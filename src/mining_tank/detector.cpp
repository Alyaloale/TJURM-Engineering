#include "mining_tank/detector.h"
#include <filesystem>
#include <opencv2/core/utils/filesystem.hpp>
#include "data_manager/control/control.h"
#include "locate/locate.h"

static int binary_ratio;
cv::Mat image_in;
cv::Mat show_contour;
cv::Mat show_triangle;
using namespace rm;
namespace fs = std::filesystem;
int count = 0;
void Control::detect_start()
{
    while(true)
    {
        //读取图片
        if(!Data::debug)
        {
            //get_image_DaHeng();
            //get_image_rgbdepth();
            get_image_RealSense();
            if(!Data::image_in_RealSense_color.empty()&&!Data::image_in_RealSense_depth.empty())
            {
                detect(Data::image_in_RealSense_color);
                //展示深度图由远到进，逐渐变蓝
                // cv::Mat depth_image_show = cv::Mat::zeros(Data::image_in_DaHeng.size(), CV_8UC3);
                // for(int i=0;i<Data::image_in_DaHeng_depth.rows;i++)
                // {
                //     for(int j=0;j<Data::image_in_DaHeng_depth.cols;j++)
                //     {
                //         float depth = Data::image_in_DaHeng_depth.at<float>(i,j);
                //         if(depth>0)
                //         {
                //             //深度值归一化到0-255
                //             float depth_normalized = std::min(depth / 5000.0f, 1.0f);
                //             //将深度值映射到颜色
                //             cv::Vec3b color = cv::Vec3b(255 * depth_normalized, 0, 255 * (1 - depth_normalized));
                //             depth_image_show.at<cv::Vec3b>(i,j) = color;
                //         }
                //     }
                // }
                // cv::imshow("depth_image_show", depth_image_show);
            }
            else
            {
                std::cout<<"image_in_RealSense_color is empty"<<std::endl;
            }
            if(Data::show_image_flag&&Data::show_depth)
            {
                cv::imshow("Aruco", Data::image_in_RealSense_depth);
            }

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
        else if(Data::debug==3)
        {
            while(true)
            {
                if(!Data::image_in_RealSense_color.empty())
                {
                    //输入空格保存图片
                    cv::imshow("Aruco", Data::image_in_RealSense_color);
                    char key = cv::waitKey(1);
                    if(key==' ')
                    {
                        cv::imwrite("/home/tjurm/Code/TJURM-Engineering/image/realsense/"+std::to_string(count)+".jpg", Data::image_in_RealSense_color);
                        count++;
                    }
                }
            }
        }
    }
}
void detect(cv::Mat &src)
{
    auto param = Param::get_instance();
    image_in = src.clone();

    std::vector<std::vector<cv::Point2f>>* mining_tank_four = new std::vector<std::vector<cv::Point2f>>;
    std::vector<cv::Point2f>* mining_tank_v = new std::vector<cv::Point2f>;


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
    StrenthenColor(src,gray_image,Data::mining_tank_color);
    if(Data::show_image_flag&&Data::show_binary_image_flag){
        cv::imshow("gray_image", gray_image);
    }

//二值化
    cv::Mat binary_image;
    if (Data::mining_tank_color == rm::ARMOR_COLOR_RED){
        binary_ratio=(*param)["Point"]["Threshold"]["RatioRed"];
    } else{
        binary_ratio=(*param)["Point"]["Threshold"]["RatioBlue"];
    }
    int threshold_from_hist = rm::getThresholdFromHist(image_in,8,binary_ratio);
    threshold_from_hist = std::clamp(threshold_from_hist, 10, 100);
    rm::getBinary(gray_image, binary_image,threshold_from_hist, rm::BINARY_METHOD_DIRECT_THRESHOLD);

//轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    show_contour = cv::Mat::zeros(image_in.size(), CV_8UC3);
    cv::findContours(binary_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //筛选获取四点轮廓
    std::vector<std::vector<cv::Point>>* mining_tank_contours_four = new std::vector<std::vector<cv::Point>>;
    MiningTankCountourFourSift(contours,show_contour, mining_tank_contours_four);
    //筛选获取V字轮廓
    std::vector<std::vector<cv::Point>>* mining_tank_contours_v = new std::vector<std::vector<cv::Point>>;
    MiningTankCountourVSift(contours,show_contour, mining_tank_contours_v);
    if(Data::show_image_flag&&Data::show_contour_flag){
        cv::imshow("show_contour", show_contour);
    }


    show_triangle = cv::Mat::zeros(image_in.size(), CV_8UC3);

    //获取四点
    if(mining_tank_contours_four->size()== 4 )GetMiningTankFour(mining_tank_contours_four,binary_image,show_triangle, mining_tank_four);
    //std::cout<<"four point contour num is "<<mining_tank_contours_four.size()<<std::endl;

    //获取V字
    if(mining_tank_contours_v->size() >=1 )GetMiningTankV(mining_tank_contours_v,binary_image,show_triangle, mining_tank_v);
    //std::cout<<"V contour num is "<<mining_tank_contours_v->size()<<std::endl;
    

    if(Data::show_image_flag&&Data::show_binary_image_flag){
        cv::imshow("binary_image", binary_image);
    }
    if(Data::show_image_flag&&Data::show_triangle_flag){
        //cv::resize(show_triangle, show_triangle, cv::Size(3840, 2160));
        cv::imshow("show_triangle", show_triangle);
    }
    if(Data::show_image_flag){
        //四点
        for(int i=0;i<mining_tank_four->size();i++){
            cv::line(image_in,(*mining_tank_four)[i][0],(*mining_tank_four)[(i+1)%(*mining_tank_four).size()][0],cv::Scalar(0,255,0),1);
        }
        if(mining_tank_four->size())cv::circle(image_in,(*mining_tank_four)[0][0],2,cv::Scalar(255,0,0),10);

        //V字
        if(mining_tank_v->size()){
            cv::line(image_in,((*mining_tank_v))[0],(*mining_tank_v)[1],cv::Scalar(255,0,0),3);
            cv::line(image_in,(*mining_tank_v)[0],(*mining_tank_v)[2],cv::Scalar(255,0,0),3);
            cv::circle(image_in,(*mining_tank_v)[0],2,cv::Scalar(255,0,0),2);
        }
    }
    //std::cout<<"mining_tank_four size: "<<mining_tank_four->size()<<std::endl;
    //std::cout<<"mining_tank_v size: "<<mining_tank_v->size()<<std::endl;
    std::pair<int,int> flag(0,0); ;
    check(mining_tank_four, mining_tank_v, flag);
    //std::cout<<"flag.first: "<<flag.first<<std::endl;
    bool islocate = false;
    islocate = locate(mining_tank_four, mining_tank_v, flag,Data::IsFourOrTwelve);
    if(Data::show_image_flag)cv::imshow("realsens",image_in);
    //释放内存
    mining_tank_four->clear();
    mining_tank_v->clear();
    mining_tank_contours_four->clear();
    mining_tank_contours_v->clear();
    delete mining_tank_contours_four;
    delete mining_tank_contours_v;
    delete mining_tank_four;
    delete mining_tank_v;
}

void check(std::vector<std::vector<cv::Point2f>>* mining_tank_four, std::vector<cv::Point2f>* mining_tank_v, std::pair<int,int> &flag)
{
    if(mining_tank_v->size() == 3)
    {
        double head = (*mining_tank_v)[0].x;
        double tail = ((*mining_tank_v)[1].x+(*mining_tank_v)[2].x)/2;
        if(head > tail)
        {
            flag.second = -1;
        }
        else
        {
            flag.second = 1;
        }
    }
    std::vector<cv::Point3d> four_point_camera;
    for(int i=0;i<mining_tank_four->size();i++)
    {
        cv::Point3f point = PixelToCameraWithoutDbscan((*mining_tank_four)[i][0], Data::image_in_RealSense_depth, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs, true);
        cv::Point3d pointd;
        pointd.x = static_cast<double>(point.x);
        pointd.y = static_cast<double>(point.y);
        pointd.z = static_cast<double>(point.z);
        double distance = cv::norm(pointd);
        //std::cout<<"pointd: "<<pointd<<" distance: "<<distance<<std::endl;
        if(point.z < 0)
        {
            flag.first = 0;
            return;
        }
        four_point_camera.push_back(pointd);
    }
    if(four_point_camera.size() == 4)
    {
        double flatness = calculateFlatness(four_point_camera);
        //std::cout<<"flatness: "<<flatness<<std::endl;
        if(flatness < 10)
        {
            flag.first = 1;
        }
        else
        {
            flag.first = 0;
        }
    }
}

bool locate(std::vector<std::vector<cv::Point2f>>* mining_tank_four, std::vector<cv::Point2f>* mining_tank_v, std::pair<int,int> flag, bool IsFourOrTwelve)
{
    std::vector<cv::Point3d> point_world;
    std::vector<cv::Point2f> point_camera;
    std::vector<cv::Point3d> point_camera_3D;
    if(flag.first == 1 && flag.second != 0)
    {
        if(!IsFourOrTwelve)
        {
            point_world.push_back(Data::points_3D[0]);
            point_world.push_back(Data::points_3D[1]);
            point_world.push_back(Data::points_3D[2]);
            point_world.push_back(Data::points_3D[3]);


            point_camera.push_back((*mining_tank_four)[0][0]);
            point_camera.push_back((*mining_tank_four)[1][0]);
            point_camera.push_back((*mining_tank_four)[2][0]);
            point_camera.push_back((*mining_tank_four)[3][0]);
            point_camera.push_back((*mining_tank_v)[0]);
            point_camera.push_back((*mining_tank_v)[1]);
            point_camera.push_back((*mining_tank_v)[2]);
        }
        else
        {
            for(int i =0; i< 4;i++)
            {
                point_world.push_back(Data::points_3D[i]);
                point_world.push_back(Data::points_3D_triangle[i][0]);
                point_world.push_back(Data::points_3D_triangle[i][1]);
                orderPointsThreeAnticlockwise((*mining_tank_four)[i]);
                for(int j = 0; j < 3; j++)
                {
                    point_camera.push_back((*mining_tank_four)[i][j]);
                }
            }
            point_camera.push_back((*mining_tank_v)[0]);
            point_camera.push_back((*mining_tank_v)[1]);
            point_camera.push_back((*mining_tank_v)[2]);
        }
        
        if(flag.second == 1)
        {
            point_world.push_back(Data::points_3D[4]);
            point_world.push_back(Data::points_3D[5]);
            point_world.push_back(Data::points_3D[6]);
        }
        else
        {
            point_world.push_back(Data::points_3D[7]);
            point_world.push_back(Data::points_3D[8]);
            point_world.push_back(Data::points_3D[9]);
        }
    }
    else if(flag.first == 1 && !flag.second)
    {
        if(!IsFourOrTwelve)
        {
            point_world.push_back(Data::points_3D[0]);
            point_world.push_back(Data::points_3D[1]);
            point_world.push_back(Data::points_3D[2]);
            point_world.push_back(Data::points_3D[3]);


            point_camera.push_back((*mining_tank_four)[0][0]);
            point_camera.push_back((*mining_tank_four)[1][0]);
            point_camera.push_back((*mining_tank_four)[2][0]);
            point_camera.push_back((*mining_tank_four)[3][0]);
        }
        else
        {
            for(int i =0; i< 4;i++)
            {
                point_world.push_back(Data::points_3D[i]);
                point_world.push_back(Data::points_3D_triangle[i][0]);
                point_world.push_back(Data::points_3D_triangle[i][1]);
                orderPointsThreeAnticlockwise((*mining_tank_four)[i]);
                for(int j = 0; j < 3; j++)
                {
                    point_camera.push_back((*mining_tank_four)[i][j]);
                }
            }
        }
    }
    else if(flag.first == 0 && flag.second != 0)
    {
        if(flag.second == 1)
        {
            point_world.push_back(Data::points_3D[4]);
            point_world.push_back(Data::points_3D[5]);
            point_world.push_back(Data::points_3D[6]);
        }
        else
        {
            point_world.push_back(Data::points_3D[7]);
            point_world.push_back(Data::points_3D[8]);
            point_world.push_back(Data::points_3D[9]);
        }
        point_camera.push_back((*mining_tank_v)[0]);
        point_camera.push_back((*mining_tank_v)[1]);
        point_camera.push_back((*mining_tank_v)[2]);
    }
    else
    {
        return false;
    }
    //std::cout<<flag.first<<" "<<flag.second<<std::endl;
    //转换为相机坐标系
    for(int i=0;i<point_camera.size();i++)
    {
        bool accept_invalid_depth = true;
        if(i >= 4&&!IsFourOrTwelve)
        {
            accept_invalid_depth = true;
        }
        else if(i >= 12 && IsFourOrTwelve)
        {
            accept_invalid_depth = false;
        }
        cv::Point3f point = PixelToCameraWithoutDbscan(point_camera[i], Data::image_in_RealSense_depth, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs, accept_invalid_depth);
        cv::Point3d pointd;
        pointd.x = static_cast<double>(-point.y);
        pointd.y = static_cast<double>(point.x);
        pointd.z = static_cast<double>(point.z);
        //右乘自定义外参旋转矩阵，绕y轴逆时针转20度
        Eigen::Matrix3d rotate_y;
        rotate_y << cos(20.0/180.0*M_PI), 0, sin(20.0/180.0*M_PI),
                    0, 1, 0,
                    -sin(20.0/180.0*M_PI), 0, cos(20.0/180.0*M_PI);
        Eigen::Vector3d point_eigen(pointd.x, pointd.y, pointd.z);
        point_eigen = rotate_y * point_eigen;
        //std::cout<<"pointd: "<<point_eigen<<std::endl;
        pointd.x = point_eigen(0);
        pointd.y = point_eigen(1);
        pointd.z = point_eigen(2);

        if(point.z < 0)
        {
            return false;
        }
        point_camera_3D.push_back(pointd);
        //std::cout<<point_world[i]<<" ";
    }
    std::cout<<std::endl;

    //转换为Eigen
    std::vector<Eigen::Vector3d> srcPoints_eigen;
    std::vector<Eigen::Vector3d> dstPoints_eigen;
    for(int i=0;i<point_world.size();i++)
    {
        Eigen::Vector3d srcPoint(point_world[i].x, point_world[i].y, point_world[i].z);
        srcPoints_eigen.push_back(srcPoint);
        Eigen::Vector3d dstPoint(point_camera_3D[i].x, point_camera_3D[i].y, point_camera_3D[i].z);
        dstPoints_eigen.push_back(dstPoint);
    }

    //计算变换矩阵
    Eigen::Matrix4d T = computeTransformMatrix(srcPoints_eigen, dstPoints_eigen);
    std::cout<<"T: "<<T<<std::endl;

    //更新共享内存
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            Data::DaHengT[i][j] = T(i,j);
        }
    }
    update_shared_data(Data::shared_data, Data::DaHengT);


    //重投影
    std::vector<cv::Point2d> pixelPoints = reprojectPointsToPixel(point_world, T, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs);

    //依次画出四个点
    for(int j=0;j<pixelPoints.size();j++)
    {
        cv::circle(Data::image_in_RealSense_color, pixelPoints[j], 5, cv::Scalar(0, 255, 0), -1);
    }
    return true;
}

void orderPointsThreeAnticlockwise(std::vector<cv::Point2f>& points)
{
    if (points.size() != 3) {
        return; // 确保只有3个点
    }
    cv::Point2f origin = points[0];
    // 1. 计算三个点的中心（几何中心）
    cv::Point2f center = (points[0] + points[1] + points[2]) / 3.0f;

    // 2. 计算每个点相对于中心的角度（逆时针方向）
    auto computeAngle = [&center](const cv::Point2f& point) {
        cv::Point2f vec = point - center;
        return std::atan2(vec.y, vec.x); // 使用 atan2 计算角度（弧度）
    };

    //3点逆时针排序
    std::sort(points.begin(), points.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
        return computeAngle(a) > computeAngle(b);
    });

    //寻找origin
    int sign = 0;
    for (int i = 0; i < points.size(); i++)
    {
        double dis = cv::norm(points[i] - origin);
        if(dis < 0.1)
        {
            sign = i;
            break;
        }
    }
    //使特殊点为第一个点，逆时针顺序不变
    if (sign != 0)
    {
        std::rotate(points.begin(), points.begin() + sign, points.end());
    }
}
