#include "locate/locate.h"
#include "locate/tools.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

double getDepthWithDbscan(std::vector<pointWithDepth>& dataset, double eps, int minpts) {
    int count = 0; // 簇计数器
    int len = dataset.size();

    // 计算每个点的邻域点
    for (int i = 0; i < len; i++) {
        for (int j = 0; j < len; j++) {
            if (i == j) continue;
            if (abs(dataset[i].depth - dataset[j].depth) < eps) {
                dataset[i].N.push_back(&dataset[j]);
            }
        }
    }

    // DBSCAN 聚类
    for (int i = 0; i < len; i++) {
        if (dataset[i].visited == 1) continue; // 跳过已访问的点
        dataset[i].visited = 1;

        if (dataset[i].N.size() >= minpts) { // 核心点
            count++;
            dataset[i].pointtype = 3; // 核心点
            dataset[i].cluster = count;

            // 扩展簇
            for (int j = 0; j < dataset[i].N.size(); j++) {
                if (dataset[i].N[j]->visited == 1) continue; // 跳过已访问的点
                dataset[i].N[j]->visited = 1;

                if (dataset[i].N[j]->N.size() >= minpts) { // 核心点
                    for (int k = 0; k < dataset[i].N[j]->N.size(); k++) {
                        dataset[i].N.push_back(dataset[i].N[j]->N[k]); // 扩展邻域
                    }
                    dataset[i].N[j]->pointtype = 3; // 核心点
                } else {
                    dataset[i].N[j]->pointtype = 2; // 边界点
                }

                if (dataset[i].N[j]->cluster == 0) {
                    dataset[i].N[j]->cluster = count; // 分配簇标签
                }
            }
        } else {
            dataset[i].pointtype = 1; // 噪声点
        }
    }

    // 计算每个簇的深度均值
    std::vector<double> cluster_depth(count + 1, 0); // 簇深度总和
    std::vector<int> cluster_count(count + 1, 0);    // 簇点数

    for (int i = 0; i < len; i++) {
        if (dataset[i].pointtype == 3) { // 只计算核心点
            cluster_depth[dataset[i].cluster] += dataset[i].depth;
            cluster_count[dataset[i].cluster]++;
        }
    }

    // 计算每个簇的深度均值
    for (int i = 1; i <= count; i++) {
        if (cluster_count[i] > 0) {
            cluster_depth[i] /= cluster_count[i];
        }
    }

    // 找到最小深度均值
    double min_depth = std::numeric_limits<double>::max();
    int min_cluster = -1;

    for (int i = 1; i <= count; i++) {
        if (cluster_count[i] > 0 && cluster_depth[i] < min_depth) {
            min_depth = cluster_depth[i];
            min_cluster = i;
        }
    }
    // for(int i=0;i<dataset.size();i++)
    // {
    //     // if(dataset[i].cluster == min_cluster)
    //     // {
    //         std::cout<<dataset[i].depth<<" ";
    //     //}
    // }
    // std::cout<<std::endl;

    // 如果没有簇，返回 NaN
    if (min_cluster == -1) {
        return 0;
    }

    return min_depth;
}


//-----------------------------------
// 基于梯度分析的空洞填充主函数
//-----------------------------------
cv::Mat gradient_based_inpainting(cv::Mat& depth_map, int max_iter) {
    CV_Assert(depth_map.type() == CV_16UC1 || depth_map.type() == CV_32F);

    // Step 1: 转换为浮点单通道格式（规范化到0-1）
    cv::Mat img;
    depth_map.convertTo(img, CV_32F);
    double min_val, max_val;
    cv::minMaxLoc(img, &min_val, &max_val);
    img = (img - min_val) / (max_val - min_val);

    // Step 2: 计算有效区域mask（假设空穴值为0）
    cv::Mat mask = (img > 0) / 255;
    mask.convertTo(mask, CV_8U);

    // Step 3: 计算梯度场（使用Sobel算子）
    cv::Mat grad_x, grad_y;
    cv::Sobel(img, grad_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(img, grad_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_REPLICATE);
    
    // Step 4: 双边滤波降噪（保留边缘）
    cv::Mat filtered;
    cv::bilateralFilter(img, filtered, 5, 0.1, 5);

    // Step 5: 构建边缘约束的空穴填充算法
    cv::Mat inpainted = filtered.clone();
    cv::Mat confidence;
    // 正确写法：分步完成克隆与类型转换
    cv::Mat temp_mask = mask.clone();  // 先克隆原始二值掩膜
    temp_mask.convertTo(confidence, CV_32F);
    const int kernel_size = 7; // 邻域搜索尺寸
    const float grad_thresh = 0.3; // 梯度阈值

    for(int iter = 0; iter < max_iter; ++iter) {
        cv::Mat boundary;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
        cv::morphologyEx(mask, boundary, cv::MORPH_GRADIENT, kernel);

        std::vector<cv::Point> boundary_pts;
        cv::findNonZero(boundary, boundary_pts);

        if(boundary_pts.empty()) break;

        // 计算每个边界点的优先级
        std::vector<std::pair<float, cv::Point>> priority_queue;
        for(auto& pt : boundary_pts) {
            cv::Rect roi(pt.x - 1, pt.y - 1, 3, 3);
            if(roi.x < 0 || roi.y < 0 || roi.x + roi.width >= img.cols || roi.y + roi.height >= img.rows) 
                continue;

            float C = cv::mean(confidence(roi))[0];
            float G = std::sqrt(grad_x.at<float>(pt)*grad_x.at<float>(pt) + 
                              grad_y.at<float>(pt)*grad_y.at<float>(pt));
            float P = C * (1.0 / (1.0 + G)); // 梯度越低优先级越高
            priority_queue.emplace_back(P, pt);
        }

        // 找到当前最高优先级点（梯度最小的位置）
        auto max_it = std::max_element(priority_queue.begin(), priority_queue.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
        cv::Point target = max_it->second;

        // 局部邻域插值（加权平均）
        cv::Rect patch_roi(target.x - kernel_size/2, target.y - kernel_size/2, kernel_size, kernel_size);
        patch_roi = patch_roi & cv::Rect(0,0,img.cols,img.rows);
        
        cv::Mat patch = filtered(patch_roi);
        cv::Mat valid = (patch > 0)/255;
        valid.convertTo(valid, CV_32F);
        
        cv::Mat weights = cv::Mat::ones(patch.size(), CV_32F); 
        weights = weights.mul(valid); // 应用有效像素权重
        float denom = cv::sum(weights)[0];
        
        if(denom > 0) {
            float interp_val = cv::sum(patch.mul(weights))[0] / denom;
            inpainted.at<float>(target) = interp_val;
            confidence.at<float>(target) = 1.0; // 更新置信度
            mask.at<uchar>(target) = 255;       // 标记为填充完成
        }
    }

    // 反向归一化
    inpainted = inpainted * (max_val - min_val) + min_val;
    inpainted.convertTo(inpainted, depth_map.type());

    return inpainted;
}