#include "mining_tank/detector.h"
#include "data_manager/param.h"
#include "mining_tank/get_Four.h"
#include "locate/locate.h"

void MiningTankCountourFourSift(std::vector<std::vector<cv::Point>> contours, cv::Mat &show_contour, std::vector<std::vector<cv::Point>>* mining_tank_contours)
{
    auto param = Param::get_instance();
    double maxhwratio = (*param)["Point"]["MiningTank"]["Four"]["Ratio"]["MaxHeightWideRatio"];
    double minwhratio = (*param)["Point"]["MiningTank"]["Four"]["Ratio"]["MinWideHeightRatio"];
    double maxarearatio = (*param)["Point"]["MiningTank"]["Four"]["Ratio"]["MaxcontourRectRatio"];
    double minarearatio = (*param)["Point"]["MiningTank"]["Four"]["Ratio"]["MincontourRectRatio"];
    double min_color_ratio = (*param)["Point"]["MiningTank"]["Four"]["Ratio"]["MinColorRatio"];
    int maxarea = (*param)["Point"]["MiningTank"]["Four"]["Area"]["MaxArea"];
    int minarea = (*param)["Point"]["MiningTank"]["Four"]["Area"]["MinArea"];

    for (auto &contour : contours)
    {
        if (contour.size() < 5)
            continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float hwratio = std::max(rect.size.width, rect.size.height) / std::min(rect.size.width, rect.size.height);
        float whratio = std::min(rect.size.height, rect.size.width) / std::max(rect.size.height, rect.size.width);
        // std::cout<<"hwratio:"<<hwratio<<std::endl;
        // std::cout<<"whratio:"<<whratio<<std::endl;

        // 获取轮廓面积
        double contour_area = cv::contourArea(contour);
        double rect_area = rect.size.width * rect.size.height;
        double area_ratio = contour_area / rect_area;
        cv::Point2f center = rect.center;
        int dis_x = std::min(center.x, show_contour.cols - center.x);
        int dis_y = std::min(center.y, show_contour.rows - center.y);
        double color_ratio = 0;
        if(Data::self_color == rm::ARMOR_COLOR_RED)
        {
            color_ratio = calculateRedness(Data::image_in_RealSense_color, contours);
        }
        else {
            color_ratio = calculateBlueness(Data::image_in_RealSense_color, contours);
        }
        // std::cout<<"area:"<<area<<std::endl;
        if (hwratio > maxhwratio ||
            whratio < minwhratio ||
            contour_area > maxarea ||
            contour_area < minarea ||
            area_ratio > maxarearatio ||
            area_ratio < minarearatio ||
            color_ratio < min_color_ratio ||
            dis_x < 20 ||
            dis_y < 20)
        {
            // std::cout << std::endl;
            // std::cout << "hwratio:" << hwratio << std::endl;
            // std::cout << "whratio:" << whratio << std::endl;
            // std::cout << "contour_area:" << contour_area << std::endl;
            // std::cout << "area_ratio:" << area_ratio << std::endl;
            // std::cout<<"color_ratio:"<<color_ratio<<std::endl;
            continue;
        }
        
        // 画出最小外接矩形
        if (Data::show_image_flag && Data::show_contour_flag)
        {
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++)
            {
                cv::line(show_contour, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
            }
        }

        // 边数筛选
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, 0.2 * sqrt(contour_area), true);
        if (polygon.size() > 4 && polygon.size() <= 9)
        {
            mining_tank_contours->push_back(contour);
        }

        // 画出多边形
        if (polygon.size() >= 4 && Data::show_image_flag && Data::show_contour_flag)
        {
            for (int i = 0; i < polygon.size(); i++)
            {
                cv::line(show_contour, polygon[i], polygon[(i + 1) % polygon.size()], cv::Scalar(0, 255, 0), 1);
            }
            cv::putText(show_contour, std::to_string(polygon.size()), polygon[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            // std::cout<<polygon.size()<<std::endl;
        }
    }
}

void GetMiningTankFour(std::vector<std::vector<cv::Point>>* contours, cv::Mat &binary_image, cv::Mat &show_triangle, std::vector<std::vector<cv::Point2f>>* mining_tank_four)
{
    std::vector<std::vector<cv::Point2f>> mining_tank_point_four;
    cv::Point2f origin;
    for (auto &contour : *contours)
    {

        // roi获取
        cv::Rect rect = cv::boundingRect(contour);
        cv::Point2i roi = cv::Point2i(rect.x, rect.y);
        roi.x = std::max(roi.x - rect.width * 1.2, 0.0);
        roi.y = std::max(roi.y - rect.height * 1.2, 0.0);
        double width = std::min(rect.width * 3.0, 1.0 * (binary_image.cols - roi.x));
        double height = std::min(rect.height * 3.0, 1.0 * (binary_image.rows - roi.y));
        cv::Mat roi_image = binary_image(cv::Rect(roi.x, roi.y, width, height));
        std::vector<std::vector<cv::Point>> roi_contours;
        int special_contours = 0;
        cv::findContours(roi_image, roi_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        for (auto &roi_c : roi_contours)
        {
            double area = cv::contourArea(roi_c);
            if (roi_c.size() <  4|| area < 4)
            continue;
            special_contours++;
        }
        //std::cout<<special_contours<<std::endl;
        // cv::imshow("roi_image",roi_image);
        // cv::waitKey(0);
        // 三角形拟合
        std::vector<cv::Point2f> contourfloat;
        for (const auto &point : contour)
        {
            contourfloat.push_back(cv::Point2f(point.x, point.y));
        }
        std::vector<cv::Point2f> triangle;
        cv::minEnclosingTriangle(contourfloat, triangle);


        // 画出三角形
        if (Data::show_image_flag && Data::show_triangle_flag)
        {
            for (int i = 0; i < triangle.size(); i++)
            {
                cv::line(show_triangle, triangle[i], triangle[(i + 1) % triangle.size()], cv::Scalar(255, 0, 0), 1);
                //cv::putText(show_triangle, std::to_string(int(triangle_camera[i].x))+" "+std::to_string(int(triangle_camera[i].y))+" "+std::to_string(int(triangle_camera[i].z)), triangle[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            }
        }

        // 角点筛选
        double min_count = 100000;
        double special_point = 0;
        for (int i = 0; i < triangle.size(); i++)
        {
            for (int j = i + 1; j < triangle.size(); j++)
            {
                cv::Point2f pointA = triangle[i], pointB = triangle[j];
                double dis = cv::norm(pointA - pointB);
                double count = CountLinePixels(pointA, pointB, binary_image);
                //std::cout<<count/dis<<std::endl;
                if (count/dis < min_count)
                {
                    min_count = count/dis;
                    for(int  k = 0;k <triangle.size();k++)
                    {
                        if(k!=i && k!=j)
                        {
                            special_point = k;
                            // cv::putText(show_triangle, std::to_string(count/dis), triangle[k], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                            // cv::circle(show_triangle, triangle[k], 2, cv::Scalar(0, 0, 255), 2);
                        }
                    }
                }
            }
        }

        // 交换特殊点和第一个点，使得特殊点为第一个点
        if(special_point)std::swap(triangle[0], triangle[special_point]);
        // 画出特殊点
        if (special_contours >= 3)
        {
            origin = cv::Point2f(triangle[0].x, triangle[0].y);
        }
        mining_tank_point_four.push_back(triangle);
    }


    //筛选四点
    std::vector<std::vector<cv::Point2f>> mining_tank_point_four_temp;
    if(mining_tank_point_four.size() >= 4)
    {
        findbestfour(mining_tank_point_four, &mining_tank_point_four_temp);
        //std::cout<<"mining_tank_point_four_temp.size():"<<mining_tank_point_four_temp.size()<<std::endl;
        if(mining_tank_point_four_temp.size() == 4)
        {
            mining_tank_point_four.clear();
            for(int i = 0; i < mining_tank_point_four_temp.size(); i++)
            {
                mining_tank_point_four.push_back(mining_tank_point_four_temp[i]);
            }
        }
        else
        {
            return;
        }
    }
    //计算中心
    cv::Point2f center = cv::Point2f(0, 0);
    for (int i = 0; i < mining_tank_point_four.size(); i++)
    {
        center += mining_tank_point_four[i][0];
    }
    center.x /= mining_tank_point_four.size();
    center.y /= mining_tank_point_four.size();

    // 四点顺时针排序
    orderPointsClockwise(mining_tank_point_four, center);
    // 寻找特殊点
    int sign = 0;
    for (int i = 0; i < mining_tank_point_four.size(); i++)
    {
        double dis = cv::norm(mining_tank_point_four[i][0] - origin);
        if(dis < 0.1)
        {
            sign = i;
            break;
        }
    }
    //使特殊点为第一个点，瞬时针顺序不变
    if (sign != 0)
    {
        std::rotate(mining_tank_point_four.begin(), mining_tank_point_four.begin() + sign, mining_tank_point_four.end());
    }


    for(int i = 0; i < mining_tank_point_four.size(); i++)
    {
        mining_tank_four->push_back(mining_tank_point_four[i]);
    }


    // 画出四点
    if (Data::show_image_flag && Data::show_triangle_flag&&mining_tank_four->size()==4)
    {
        for (int i = 0; i < mining_tank_four->size(); i++)
        {
            cv::circle(show_triangle, (*mining_tank_four)[i][0], 2, cv::Scalar(0, 255, 0), 2);
            cv::putText(show_triangle, std::to_string(i), (*mining_tank_four)[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
        }
    }
}

int CountLinePixels(cv::Point p1, cv::Point p2, cv::Mat &binary_image) {
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    int count = 0;
    while (true) {
        //遍历周围9个点
        for(int i = -1; i < 2; i++)
        {
            for(int j = -1; j < 2; j++)
            {
                if(x1+i < 0 || x1+i >= binary_image.cols ||
                   y1+j < 0 || y1+j >= binary_image.rows)
                {
                    continue;
                }
                if(binary_image.at<uchar>(y1+i, x1+j) == 255)
                {
                    count+=1;
                }
            }
        }

        if (x1 == x2 && y1 == y2) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    return count;
}


double calculateRedness(cv::Mat& src, const std::vector<std::vector<cv::Point>>& contours) {
    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED); // 填充所有轮廓内部[1,2](@ref)
    double color_ratio = 0;
    int count = 0;
    const double MAX_DISTANCE = 441.67295593; // 255*sqrt(3)
    for (int y = 0; y < mask.rows; y++) {
        for (int x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) == 255) {
                int B = src.at<cv::Vec3b>(y, x)[0];
                int G = src.at<cv::Vec3b>(y, x)[1];
                int R = src.at<cv::Vec3b>(y, x)[2];
                double distance = sqrt(pow(255 - R, 2) + pow(G, 2) + pow(B, 2));
                color_ratio += 1.0 - (distance / MAX_DISTANCE);
                count++;
            }
        }
    }
    return color_ratio / count;
}


double calculateBlueness(cv::Mat& src, const std::vector<std::vector<cv::Point>>& contours) {
    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED); // 填充所有轮廓内部[1,2](@ref)
    double color_ratio = 0;
    int count = 0;
    const double MAX_DISTANCE = 441.67295593; // 255*sqrt(3)
    for (int y = 0; y < mask.rows; y++) {
        for (int x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) == 255) {
                int B = src.at<cv::Vec3b>(y, x)[0];
                int G = src.at<cv::Vec3b>(y, x)[1];
                int R = src.at<cv::Vec3b>(y, x)[2];
                double distance = sqrt(pow(R, 2) + pow(G, 2) + pow(255 - B, 2));
                color_ratio += 1.0 - (distance / MAX_DISTANCE);
                count++;
            }
        }
    }
    return color_ratio / count;
}


void orderPointsClockwise(std::vector<std::vector<cv::Point2f>> &points , cv::Point2f &centroid) {

    // Step 1: 定义排序规则（顺时针）
    auto compare = [&centroid](const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b) {
        // 计算相对向量
        cv::Point2f vecA = a[0] - centroid;
        cv::Point2f vecB = b[0] - centroid;
        
        // 计算极角并比较
        float angleA = atan2(vecA.y, vecA.x);
        float angleB = atan2(vecB.y, vecB.x);
        return angleA > angleB; // 降序排序实现顺时针
    };

    // Step 2: 排序并返回结果
    std::sort(points.begin(), points.end(), compare);
}

void findbestfour(std::vector<std::vector<cv::Point2f>> &fours, std::vector<std::vector<cv::Point2f>>* best_four)
{
    //从点集中找出四个点不在乎顺序的组合
    double min_ratio = 1000000;
    for (int i = 0; i < fours.size(); i++)
    {
        for (int j = i + 1; j < fours.size(); j++)
        {
            for (int k = j + 1; k < fours.size(); k++)
            {
                for (int l = k + 1; l < fours.size(); l++)
                {
                    std::vector<cv::Point2f> four_temp;
                    four_temp.push_back(fours[i][0]);
                    four_temp.push_back(fours[j][0]);
                    four_temp.push_back(fours[k][0]);
                    four_temp.push_back(fours[l][0]);
                    std::vector<cv::Point3f> four_temp_camera;
                    for(int m = 0; m < four_temp.size(); m++)
                    {
                        cv::Point3f point = PixelToCameraWithoutDbscan(four_temp[m], Data::image_in_RealSense_depth, Data::realsense_camera.intrinsic_matrix, Data::realsense_camera.distortion_coeffs, true);
                        // if(point.z < 0)
                        // {
                        //     continue;
                        // }
                        four_temp_camera.push_back(point);
                    }
                    if(four_temp_camera.size() == 4)
                    {
                        double max_distance = 0, min_distance = 1000000;
                        for(int m = 0; m < four_temp_camera.size(); m++)
                        {
                            for(int n = m + 1; n < four_temp_camera.size(); n++)
                            {
                                double distance = cv::norm(four_temp_camera[m] - four_temp_camera[n]);
                                max_distance = std::max(max_distance, distance);
                                min_distance = std::min(min_distance, distance);
                                //std::cout<<"distance: "<<distance<<std::endl;
                            }
                        }
                        //std::cout<<four_temp_camera[0]<<" "<<four_temp_camera[1]<<" "<<four_temp_camera[2]<<" "<<four_temp_camera[3]<<std::endl;
                        double ratio = 0;
                        //ratio = std::abs(max_distance * min_distance / 112000 - 1);
                        //std::cout<<"ratio: "<<ratio<<std::endl;
                        if(ratio < min_ratio&& ratio < 0.5)
                        {
                            min_ratio = ratio;
                            best_four->clear();
                            best_four->push_back(fours[i]);
                            best_four->push_back(fours[j]);
                            best_four->push_back(fours[k]);
                            best_four->push_back(fours[l]);
                        }
                    }
                }
            }
        }
    }
}