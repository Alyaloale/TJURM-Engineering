#include "data_manager/base.h"
#include "data_manager/param.h"
#include "mining_tank/detector.h"
#include "mining_tank/get_V.h"
#include "locate/locate.h"


void MiningTankCountourVSift(std::vector<std::vector<cv::Point>> contours,cv::Mat &show_contour, std::vector<std::vector<cv::Point>>* mining_tank_contours)
{
    auto param = Param::get_instance();
    double maxhwratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MaxHeightWideRatio"];
    double minhwratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MinHeightWideRatio"];
    double maxwhratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MaxWideHeightRatio"];
    double minwhratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MinWideHeightRatio"];
    double maxarearatio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MaxcontourRectRatio"];
    double minarearatio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MincontourRectRatio"];
    int maxarea = (*param)["Point"]["MiningTank"]["V"]["Area"]["MaxArea"];
    int minarea = (*param)["Point"]["MiningTank"]["V"]["Area"]["MinArea"];
    double maxrectarea = 0;
    for (auto &contour : contours) {
        if(contour.size()<5)continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float hwratio = std::max(rect.size.width, rect.size.height) / std::min(rect.size.width, rect.size.height);
        float whratio = std::min(rect.size.height, rect.size.width) / std::max(rect.size.height, rect.size.width);
        //std::cout<<"hwratio:"<<hwratio<<std::endl;
        //std::cout<<"whratio:"<<whratio<<std::endl;


        //获取轮廓面积
        double contour_area = cv::contourArea(contour);
        double rect_area = rect.size.width*rect.size.height;
        double area_ratio = contour_area/rect_area;
        //std::cout<<"area:"<<area<<std::endl;

        if(contour_area>maxarea
        ||contour_area<minarea
        ||area_ratio>maxarearatio
        ||area_ratio<minarearatio
        ||hwratio<minhwratio
        ||hwratio>maxhwratio
        ||whratio<minwhratio
        ||whratio>maxwhratio
        ){
            //输出参数
            if(rect_area  > 300)
            {
                // std::cout<<std::endl;
                // std::cout<< rect_area<<std::endl;
                // std::cout<<"hwration:"<<hwratio<<std::endl;
                // std::cout<<"whration:"<<whratio<<std::endl;
                // std::cout<<"area:"<<contour_area<<std::endl;
                // std::cout<<"area_ratio:"<<area_ratio<<std::endl;
            }
            continue;
        }
        // std::cout<<std::endl;
        // std::cout<< rect_area<<std::endl;
        // std::cout<<"hwration:"<<hwratio<<std::endl;
        // std::cout<<"whration:"<<whratio<<std::endl;
        // std::cout<<"area:"<<contour_area<<std::endl;
        // std::cout<<"area_ratio:"<<area_ratio<<std::endl;
        if(rect_area > maxrectarea)
        {
            if(maxrectarea)std::swap((*mining_tank_contours)[0],(*mining_tank_contours)[1]);
            maxrectarea = rect_area;
        }
        else {
            continue;
        }
        //画出最小外接矩形
        if(Data::show_image_flag&&Data::show_contour_flag){
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++) {
            cv::line(show_contour, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 1);
            }
        }


        //边数筛选
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, 0.005 * contour_area, true);
        if(polygon.size() >= 4&&polygon.size() <= 6){
            mining_tank_contours->push_back(contour);
        }
        //std::cout<<mining_tank_contours->size()<<std::endl;

        //画出多边形
        if(polygon.size() >= 4&&Data::show_image_flag&&Data::show_contour_flag){
            for(int i=0;i<polygon.size();i++){
                cv::line(show_contour,polygon[i],polygon[(i+1)%polygon.size()],cv::Scalar(0,0,255),1);
            }
            cv::putText(show_contour,std::to_string(polygon.size()),polygon[0],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);
            //std::cout<<polygon.size()<<std::endl;
        }
    }
}


void GetMiningTankV(std::vector<std::vector<cv::Point>>* contours,cv::Mat &binary_image,cv::Mat& show_triangle,std::vector<cv::Point2f>* mining_tank_v)
{
    cv::Point2f orignin;

    auto contour = (*contours)[0];
   //三角形拟合
    std::vector<cv::Point2f> contourfloat;
    for (const auto& point : contour){
        contourfloat.push_back(cv::Point2f(point.x, point.y));
    }
    std::vector<cv::Point2f> triangle;
    cv::minEnclosingTriangle(contourfloat,triangle);
    //画出三角形
    if(Data::show_image_flag&&Data::show_triangle_flag){
        for(int i=0;i<triangle.size();i++){
            cv::line(show_triangle,triangle[i],triangle[(i+1)%triangle.size()],cv::Scalar(255,0,0),1);
        }
    }


    //角点筛选
    int max_count = 0;
    int special_point = 0;
    for(int i=0;i<triangle.size();i++){
        int count = 0;
        cv::Point2f point = triangle[i];
        cv::RotatedRect rect = cv::minAreaRect(contour);
        double r = std::min(rect.size.width,rect.size.height)/2;
        //遍历半径为r范围内的点
        for(int j = point.x-r;j<point.x+r;j++){
            for(int k = point.y-r;k<point.y+r;k++){
                if(j<0||j>=binary_image.cols||k<0||k>=binary_image.rows)continue;
                if(binary_image.at<uchar>(k,j)==255){
                    count++;
                }
            }
        }
        if(count>max_count){
            max_count = count;
            special_point = i;
        }
    }


    //交换特殊点和第一个点，使得特殊点为第一个点
    std::swap(triangle[0],triangle[special_point]);
    if(triangle[2].y < triangle[1].y){
        std::swap(triangle[1],triangle[2]);
    }
    for(int i=0;i<triangle.size();i++){
        mining_tank_v->push_back(triangle[i]);
    }


    std::vector<cv::Point3d> triangle_camera;
    for(int i=0;i<triangle.size();i++){
        cv::Point3f point = PixelToCameraWithoutDbscan(triangle[i], Data::image_in_RealSense_depth, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs, false);
        cv::Point3d point3d = cv::Point3d(point.x,point.y,point.z);
        //std::cout<<"point3d: "<<point3d<<std::endl;
        if(point.z < 0){
            continue;
        }
        triangle_camera.push_back(point3d);
    }
    if(triangle_camera.size() == 3){
        //计算三角形面积
        double parallelogramArea = triangleArea3D(triangle_camera[0], triangle_camera[1], triangle_camera[2]);
        //std::cout<<"parallelogramArea: "<<parallelogramArea<<std::endl;
        double real_area_ratio = std::abs(1 - parallelogramArea / 9800);
        //std::cout<<"real_area_ratio: "<<real_area_ratio<<std::endl;
        if(real_area_ratio > 0.3)
        {
                mining_tank_v->clear();
        }
    }

    //画出顶点
    if(Data::show_image_flag&&Data::show_triangle_flag){
        cv::circle(show_triangle,triangle[0],2,cv::Scalar(0,255,0),2);
    }
}


void StrenthenColor(cv::Mat &src, cv::Mat &dst, rm::ArmorColor color)
{
    //分离通道
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    cv::Mat b = channels[0];
    cv::Mat g = channels[1];
    cv::Mat r = channels[2];
    if(color == rm::ARMOR_COLOR_RED)
    {
        //红色-蓝色
        cv::subtract(b, r, dst);
    }
    else if(color == rm::ARMOR_COLOR_BLUE)
    {
        //蓝色
        cv::subtract(r, b, dst);
    }
    else
    {
        std::cout<<"color error"<<std::endl;
    }
}

double triangleArea3D(const cv::Point3d& A, const cv::Point3d& B, const cv::Point3d& C) {
    cv::Point3d AB = B - A; // 向量 AB
    cv::Point3d AC = C - A; // 向量 AC

    // 计算叉积 AB × AC
    cv::Point3d crossProduct(
        AB.y * AC.z - AB.z * AC.y,  // x 分量
        AB.z * AC.x - AB.x * AC.z,  // y 分量
        AB.x * AC.y - AB.y * AC.x   // z 分量
    );

    // 叉积的模长（即平行四边形的面积）
    double parallelogramArea = cv::norm(crossProduct);

    // 三角形面积 = 平行四边形面积 / 2
    return 0.5 * parallelogramArea;
}