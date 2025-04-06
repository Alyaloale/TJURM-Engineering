#ifndef LOCATE_TOOLS_H
#define LOCATE_TOOLS_H
#include "data_manager/param.h"
#include "data_manager/base.h"
#include <Eigen/Dense>


class pointWithDepth {
    public:
        double depth;
        cv::Point2f pos;
        int pointtype;//1 noise 2 border 3 core
        int visited;
        int cluster;
        std::vector<pointWithDepth*>N; //point指针的向量
        pointWithDepth(){
            pointtype = 1;
            visited = 0;
            cluster = 0;
        }
    };
double getDepthWithDbscan(std::vector<pointWithDepth>& dataset, double eps, int minpts);//dbscan算法获取深度值
cv::Mat gradient_based_inpainting(cv::Mat& depth_map, int max_iter = 50);//基于梯度分析的空洞填充


#endif