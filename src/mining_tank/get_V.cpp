#include "data_manager/base.h"
#include "data_manager/param.h"
#include "mining_tank/detector.h"
#include "mining_tank/get_V.h"


std::vector<std::vector<cv::Point>> MiningTankCountourVSift(std::vector<std::vector<cv::Point>> &contours,cv::Mat& show_contour)
{
    auto param = Param::get_instance();
    std::vector<std::vector<cv::Point>> mining_tank_contours;
    double maxhwratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MaxHeightWideRatio"];
    double minhwratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MinHeightWideRatio"];
    double maxwhratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MaxWideHeightRatio"];
    double minwhratio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MinWideHeightRatio"];
    double maxarearatio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MaxcontourRectRatio"];
    double minarearatio = (*param)["Point"]["MiningTank"]["V"]["Ratio"]["MincontourRectRatio"];
    int maxarea = (*param)["Point"]["MiningTank"]["V"]["Area"]["MaxArea"];
    int minarea = (*param)["Point"]["MiningTank"]["V"]["Area"]["MinArea"];

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

        if(hwratio>maxhwratio
        ||hwratio<minhwratio
        ||whratio>maxwhratio
        ||whratio<minwhratio
        ||contour_area>maxarea
        ||contour_area<minarea
        ||area_ratio>maxarearatio
        ||area_ratio<minarearatio
        ){
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
        if(polygon.size() >= 4&&polygon.size() <= 8){
            mining_tank_contours.push_back(contour);
        }


        //画出多边形
        if(polygon.size() >= 4&&Data::show_image_flag&&Data::show_contour_flag){
            for(int i=0;i<polygon.size();i++){
                cv::line(show_contour,polygon[i],polygon[(i+1)%polygon.size()],cv::Scalar(0,0,255),1);
            }
            cv::putText(show_contour,std::to_string(polygon.size()),polygon[0],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);
            //std::cout<<polygon.size()<<std::endl;
        }
    }
    return mining_tank_contours;
}


MiningTankV GetMiningTankV(std::vector<std::vector<cv::Point>> contours,cv::Mat binary_image,cv::Mat& show_triangle)
{
    MiningTankV mining_tank_v;
    cv::Point2f orignin;
    for (auto &contour : contours) {


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
        for(int i=0;i<triangle.size();i++){
            mining_tank_v.point.push_back(triangle[i]);
        }


        //画出顶点
        if(Data::show_image_flag&&Data::show_triangle_flag){
            cv::circle(show_triangle,triangle[0],2,cv::Scalar(0,255,0),2);
        }
    }
    return mining_tank_v;
}


void StrenthenColor(cv::Mat &src, cv::Mat &dst, rm::ArmorColor color)
{
    //分离通道
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    if(color == rm::ARMOR_COLOR_RED){
        cv::subtract(channels[2],channels[0],dst);
    }
    else if(color == rm::ARMOR_COLOR_BLUE){
        cv::subtract(channels[0],channels[2],dst);
    }
    else{
        std::cout<<"strenten color is not red or blue"<<std::endl;
        return;
    }
    dst = cv::Mat::zeros(src.size(), CV_8UC1);
    int b ,g,r,target,other1,other2;
    if(color == rm::ARMOR_COLOR_RED){
        b = 0;
        g = 0;
        r = 255;
        target = 2;
        other1 = 0;
        other2 = 1;
    }
    else if(color == rm::ARMOR_COLOR_BLUE){
        b = 255;
        g = 0;
        r = 0;
        target = 0;
        other1 = 2;
        other2 = 1;
    }
    else{
        std::cout<<"strenten color is not red or blue"<<std::endl;
        return;
    }
    //增强颜色
    for(int i=0;i<src.rows;i++){
        for(int j=0;j<src.cols;j++){
            double dis = std::sqrt((channels[0].at<uchar>(i,j)-b)*(channels[0].at<uchar>(i,j)-b)+(channels[1].at<uchar>(i,j)-g)*(channels[1].at<uchar>(i,j)-g)+(channels[2].at<uchar>(i,j)-r)*(channels[2].at<uchar>(i,j)-r));
            double x=sqrt(dis/255);
            int gap = 2*channels[target].at<uchar>(i,j)-channels[other1].at<uchar>(i,j)-channels[other2].at<uchar>(i,j);
            gap =gap>0?gap:0;
            dst.at<uchar>(i,j) = (1-std::pow(x,2.5)+std::pow(2.718,-20*x))/4*255+channels[target].at<uchar>(i,j)*0.25+gap*0.125;
        }
    }
}