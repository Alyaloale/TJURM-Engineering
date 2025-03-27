#include "locate/locate.h"
#include "locate/dbscan.h"
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