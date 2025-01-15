#include "mining_tank/detector.h"
#include "data_manager/param.h"
#include "mining_tank/get_Four.h"


std::vector<std::vector<cv::Point>> MiningTankCountourFourSift(std::vector<std::vector<cv::Point>> &contours,cv::Mat& show_contour)
{
    auto param = Param::get_instance();
    std::vector<std::vector<cv::Point>> mining_tank_contours;
    double maxhwratio = (*param)["Points"]["MiningTank"]["Four"]["Ratio"]["MaxHeightWideRatio"];
    double minwhratio = (*param)["Points"]["MiningTank"]["Four"]["Ratio"]["MinWideHeightRatio"];
    double maxarearatio = (*param)["Points"]["MiningTank"]["Four"]["Ratio"]["MaxcontourRectRatio"];
    double minarearatio = (*param)["Points"]["MiningTank"]["Four"]["Ratio"]["MincontourRectRatio"];
    int maxarea = (*param)["Points"]["MiningTank"]["Four"]["Area"]["MaxArea"];
    int minarea = (*param)["Points"]["MiningTank"]["Four"]["Area"]["MinArea"];

    for (auto &contour : contours) {
        if(contour.size()<5)continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float hwratio = std::max(rect.size.width, rect.size.height) / std::min(rect.size.width, rect.size.height);
        float whratio = std::min(rect.size.height, rect.size.width) / std::max(rect.size.height, rect.size.width);
        // std::cout<<"hwratio:"<<hwratio<<std::endl;
        // std::cout<<"whratio:"<<whratio<<std::endl;


        //获取轮廓面积
        double contour_area = cv::contourArea(contour);
        double rect_area = rect.size.width*rect.size.height;
        double area_ratio = contour_area/rect_area;
        cv::Point2f center = rect.center;
        int dis_x=std::min(center.x,show_contour.cols-center.x);
        int dis_y=std::min(center.y,show_contour.rows-center.y);
        //std::cout<<"area:"<<area<<std::endl;
        if(hwratio>maxhwratio
        ||whratio<minwhratio
        ||contour_area>maxarea
        ||contour_area<minarea
        ||area_ratio>maxarearatio
        ||area_ratio<minarearatio
        ||dis_x<20
        ||dis_y<20
        ){
            std::cout<<std::endl;
        std::cout<<"hwratio:"<<hwratio<<std::endl;
        std::cout<<"whratio:"<<whratio<<std::endl;
        std::cout<<"contour_area:"<<contour_area<<std::endl;
        std::cout<<"area_ratio:"<<area_ratio<<std::endl;
            continue;
        }

        //画出最小外接矩形
        if(Data::show_image_flag&&Data::show_contour_flag){
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++) {
            cv::line(show_contour, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
            }
        }


        //边数筛选
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, 0.2 * sqrt(contour_area), true);
        if(polygon.size() >= 3&&polygon.size() <= 9){
            mining_tank_contours.push_back(contour);
        }


        //画出多边形
        if(polygon.size() >= 4&&Data::show_image_flag&&Data::show_contour_flag){
            for(int i=0;i<polygon.size();i++){
                cv::line(show_contour,polygon[i],polygon[(i+1)%polygon.size()],cv::Scalar(0,255,0),1);
            }
            cv::putText(show_contour,std::to_string(polygon.size()),polygon[0],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);
            //std::cout<<polygon.size()<<std::endl;
        }
    }
    return mining_tank_contours;
}


MiningTankFour GetMiningTankFour(std::vector<std::vector<cv::Point>> contours,cv::Mat &binary_image,cv::Mat& show_triangle)
{
    MiningTankFour mining_tank_four;
    std::vector<std::vector<cv::Point2f>> mining_tank_point_four;
    cv::Point2f orignin;
    for (auto &contour : contours) {


        //roi获取
        cv::Rect rect = cv::boundingRect(contour);
        cv::Point2i roi = cv::Point2i(rect.x,rect.y);
        roi.x = std::max(roi.x-rect.width*0.8,0.0);
        roi.y = std::max(roi.y-rect.height*0.8,0.0);
        double width = std::min(rect.width*3.0,1.0*(binary_image.cols-roi.x));
        double height = std::min(rect.height*3.0,1.0*(binary_image.rows-roi.y));
        cv::Mat roi_image = binary_image(cv::Rect(roi.x,roi.y,width,height));
        // cv::imshow("roi_image",roi_image);
        // cv::waitKey(0);
        std::vector<std::vector<cv::Point>> roi_contours,special_contours;
        cv::findContours(roi_image, roi_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        for(auto &roi_c:roi_contours){
            double area = cv::contourArea(roi_c);
            if(roi_c.size()<5||area<8)continue;
            special_contours.push_back(roi_c);
        }

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
        double min_count = 100000;
        int special_point = 0;
        for(int i=0;i<triangle.size();i++){
            for(int j=i+1;j<triangle.size();j++)
            {
                cv::Point2f pointA = triangle[i],pointB = triangle[j];
                cv::Point2f pointC = (pointA+pointB)/2;
                int r = 2;
                int count = 0;
                for(int k = pointC.x-r;k<pointC.x+r;k++){
                    for(int l = pointC.y-r;l<pointC.y+r;l++){
                        if(k<0||k>=binary_image.cols||l<0||l>=binary_image.rows)continue;
                        if(binary_image.at<uchar>(l,k)==255){
                            count++;
                        }
                    }
                }
                if(count<=min_count)
                {
                    min_count = count;
                    for(int k=0;k<triangle.size();k++)
                    {
                        if(k!=i&&k!=j)
                        {
                            special_point = k;
                            break;
                        }
                    }
                }
            }
        }


        //交换特殊点和第一个点，使得特殊点为第一个点
        std::swap(triangle[0],triangle[special_point]);
        //std::cout<<special_contours.size()<<std::endl;
        if(special_contours.size()>=3){

            //获取中心和面积
            std::vector<std::pair<cv::Point2f,double>>center_area;
            for(auto &contour_mini:special_contours)
            {
                cv::RotatedRect rect = cv::minAreaRect(contour_mini);
                cv::Point2f center = rect.center;
                double area = cv::contourArea(contour_mini);
                center_area.push_back(std::make_pair(center,area));
            }


            //根据面积排序从小到大
            std::sort(center_area.begin(),center_area.end(),[](std::pair<cv::Point2f,double> a,std::pair<cv::Point2f,double> b){
                return a.second<b.second;
            });

            //依据到两个小轮廓的距离筛选出顶点
            cv::Point2f A = center_area[0].first;
            cv::Point2f B = center_area[1].first;
            //修正顶点
            A.x+=roi.x;
            A.y+=roi.y;
            B.x+=roi.x;
            B.y+=roi.y;
            double maxABdis = 0;
            int C =0;
            for(int i=0;i<triangle.size();i++){
                cv::Point2f point = triangle[i];
                double dis1 = cv::norm(A-point);
                double dis2 = cv::norm(B-point);
                if(std::min(dis1,dis2)>maxABdis){
                    maxABdis = std::min(dis1,dis2);
                    C = i;
                }
            }
            std::swap(triangle[0],triangle[C]);
            orignin = triangle[0];
        }
        mining_tank_point_four.push_back(triangle);
    }


    //计算中心
    cv::Point2f center;
    for(int i=0;i<mining_tank_point_four.size();i++){
        center += mining_tank_point_four[i][0];
    }
    center.x /= mining_tank_point_four.size();
    center.y /= mining_tank_point_four.size();
    mining_tank_four.center = center;


    //顺时针排序2143相限
    std::sort(mining_tank_point_four.begin(),mining_tank_point_four.end(),[center](std::vector<cv::Point2f> a,std::vector<cv::Point2f> b){
        return atan2(a[0].y-center.y,a[0].x-center.x)>atan2(b[0].y-center.y,b[0].x-center.x);
    });
    int flag = 0;
    for(int i=0;i<mining_tank_point_four.size();i++){
        double dis = cv::norm(mining_tank_point_four[i][0]-orignin);
        if(dis<0.1){
            flag = i;
        }
    }
    for(int i = flag;i<mining_tank_point_four.size();i++){
        mining_tank_four.point.push_back(mining_tank_point_four[i]);
    }
    for(int i = 0;i<flag;i++){
        mining_tank_four.point.push_back(mining_tank_point_four[i]);
    }


    //画出四点
    if(Data::show_image_flag&&Data::show_triangle_flag){
        for(int i=0;i<mining_tank_four.point.size();i++){
            cv::circle(show_triangle,mining_tank_four.point[i][0],2,cv::Scalar(0,255,0),2);
        }
    }

    return mining_tank_four;
}