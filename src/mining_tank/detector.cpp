#include "mining_tank/detector.h"


static int binary_ratio;
static bool show_image_flag = false;
static bool show_binary_image_flag = false;
static bool show_contour_flag = false;
static bool show_triangle_flag = false;
cv::Mat image;
cv::Mat show_contour;
cv::Mat show_triangle;
void detect(cv::Mat &src)
{
    auto param = Param::get_instance();
    show_image_flag = (*param)["Debug"]["ShowImage"];
    show_binary_image_flag = (*param)["Debug"]["BinaryImage"];
    show_contour_flag = (*param)["Debug"]["ShowContour"];
    show_triangle_flag = (*param)["Debug"]["ShowTriangle"];
    image = src.clone();

    MiningTankFour mining_tank_four;
    MiningTankV mining_tank_v;


//灰度化二值化
    cv::Mat gray_image;
    //rm::getGrayScale(image, gray_image, Data::mining_tank_color, rm::GRAY_SCALE_METHOD_RGB);
    StrenthenColor(image,gray_image,Data::mining_tank_color);

    cv::Mat binary_image;
    if (Data::mining_tank_color == rm::ARMOR_COLOR_RED){
        binary_ratio=(*param)["Points"]["Threshold"]["RatioRed"];
    } else{
        binary_ratio=(*param)["Points"]["Threshold"]["RatioBlue"];
    }
    int threshold_from_hist = rm::getThresholdFromHist(image,8,binary_ratio);
    threshold_from_hist = std::clamp(threshold_from_hist, 10, 100);
    rm::getBinary(gray_image, binary_image,threshold_from_hist, rm::BINARY_METHOD_DIRECT_THRESHOLD);


//轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    show_contour = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::findContours(binary_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //筛选获取四点轮廓
    std::vector<std::vector<cv::Point>> mining_tank_contours_four = MiningTankCountourFourSift(contours);
    //筛选获取V字轮廓
    std::vector<std::vector<cv::Point>> mining_tank_contours_v = MiningTankCountourVSift(contours);
    //GetMaxTwoLine(mining_tank_contours_four,binary_image);
    if(show_image_flag&&show_contour_flag){
        cv::imshow("show_contour", show_contour);
    }


    show_triangle = cv::Mat::zeros(image.size(), CV_8UC3);

    //获取四点
    if(mining_tank_contours_four.size()==4){
        mining_tank_four = GetMiningTankFour(mining_tank_contours_four,binary_image);
    }
    else if(mining_tank_contours_four.size()>4){
        std::cout<<"four point contour num is more than 4"<<std::endl;
    }
    else{
        std::cout<<"four point contour num is less than 4"<<std::endl;
        std::cout<<"four point contour num is "<<mining_tank_contours_four.size()<<std::endl;
    }

    //获取V字
    if(mining_tank_contours_v.size()==1){
        mining_tank_v = GetMiningTankV(mining_tank_contours_v,binary_image);
    }
    else if(mining_tank_contours_v.size()>2){
        std::cout<<"V contour num is more than 1"<<std::endl;
    }
    else{
        std::cout<<"V contour is missing"<<std::endl;
        std::cout<<"V contour num is "<<mining_tank_contours_v.size()<<std::endl;
    }

    //画出四点矩形
    if(show_image_flag&&mining_tank_contours_four.size()==4){
        for(int i=0;i<mining_tank_four.point.size();i++){
            cv::line(image,mining_tank_four.point[i][0],mining_tank_four.point[(i+1)%mining_tank_four.point.size()][0],cv::Scalar(0,255,0),1);
        }
         cv::circle(image,mining_tank_four.point[0][0],2,cv::Scalar(255,0,0),2);
    }
    //画出V字
    if(show_image_flag&&mining_tank_contours_v.size()==1){
        cv::line(image,mining_tank_v.point[0],mining_tank_v.point[1],cv::Scalar(0,255,0),1);
        cv::line(image,mining_tank_v.point[0],mining_tank_v.point[2],cv::Scalar(0,255,0),1);
        cv::circle(image,mining_tank_v.point[0],2,cv::Scalar(255,0,0),2);
    }
    if(show_image_flag&&show_binary_image_flag){
        cv::imshow("gray_image", gray_image);
        cv::imshow("binary_image", binary_image);
    }
    if(show_image_flag&&show_triangle_flag){
        cv::imshow("show_triangle", show_triangle);
    }
    if(show_image_flag){
        cv::imshow("image", image);
    }
     cv::waitKey(1);
    //如果输入q下一张
}

std::vector<std::vector<cv::Point>> MiningTankCountourFourSift(std::vector<std::vector<cv::Point>> &contours)
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
        int dis_x=std::min(center.x,image.cols-center.x);
        int dis_y=std::min(center.y,image.rows-center.y);
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
        if(show_image_flag&&show_contour_flag){
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
        if(polygon.size() >= 4&&show_image_flag&&show_contour_flag){
            for(int i=0;i<polygon.size();i++){
                cv::line(show_contour,polygon[i],polygon[(i+1)%polygon.size()],cv::Scalar(0,255,0),1);
            }
            cv::putText(show_contour,std::to_string(polygon.size()),polygon[0],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);
            //std::cout<<polygon.size()<<std::endl;
        }
    }
    return mining_tank_contours;
}


std::vector<std::vector<cv::Point>> MiningTankCountourVSift(std::vector<std::vector<cv::Point>> &contours)
{
    auto param = Param::get_instance();
    std::vector<std::vector<cv::Point>> mining_tank_contours;
    double maxhwratio = (*param)["Points"]["MiningTank"]["V"]["Ratio"]["MaxHeightWideRatio"];
    double minhwratio = (*param)["Points"]["MiningTank"]["V"]["Ratio"]["MinHeightWideRatio"];
    double maxwhratio = (*param)["Points"]["MiningTank"]["V"]["Ratio"]["MaxWideHeightRatio"];
    double minwhratio = (*param)["Points"]["MiningTank"]["V"]["Ratio"]["MinWideHeightRatio"];
    double maxarearatio = (*param)["Points"]["MiningTank"]["V"]["Ratio"]["MaxcontourRectRatio"];
    double minarearatio = (*param)["Points"]["MiningTank"]["V"]["Ratio"]["MincontourRectRatio"];
    int maxarea = (*param)["Points"]["MiningTank"]["V"]["Area"]["MaxArea"];
    int minarea = (*param)["Points"]["MiningTank"]["V"]["Area"]["MinArea"];

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
        if(show_image_flag&&show_contour_flag){
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
        if(polygon.size() >= 4&&show_image_flag&&show_contour_flag){
            for(int i=0;i<polygon.size();i++){
                cv::line(show_contour,polygon[i],polygon[(i+1)%polygon.size()],cv::Scalar(0,0,255),1);
            }
            cv::putText(show_contour,std::to_string(polygon.size()),polygon[0],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);
            //std::cout<<polygon.size()<<std::endl;
        }
    }
    return mining_tank_contours;
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


MiningTankFour GetMiningTankFour(std::vector<std::vector<cv::Point>> contours,cv::Mat &binary_image)
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
        if(show_image_flag&&show_triangle_flag){
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
    if(show_image_flag&&show_triangle_flag){
        for(int i=0;i<mining_tank_four.point.size();i++){
            cv::circle(show_triangle,mining_tank_four.point[i][0],2,cv::Scalar(0,255,0),2);
        }
    }

    return mining_tank_four;
}

MiningTankV GetMiningTankV(std::vector<std::vector<cv::Point>> contours,cv::Mat binary_image)
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
        if(show_image_flag&&show_triangle_flag){
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
        if(show_image_flag&&show_triangle_flag){
            cv::circle(show_triangle,triangle[0],2,cv::Scalar(0,255,0),2);
        }
    }
    return mining_tank_v;
}
void StrenthenColor(cv::Mat &src, cv::Mat &dst, rm::ArmorColor color)
{
    //分离通道
    std::vector<cv::Mat> channels;
    dst = cv::Mat::zeros(src.size(),CV_8UC1);
    cv::split(src, channels);
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
        other1 = 1;
        other2 = 2;
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