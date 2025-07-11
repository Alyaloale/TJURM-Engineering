# TJURM Engineering - 工程机器人视觉系统

## 项目简介

这是一个基于OpenCV和RealSense相机/大恒相机的工程车视觉识别系统，主要用于RoboMaster比赛中的工程机器人任务。系统能够检测和定位矿物兑换台的四个/十二个角点和V字标记，实现精准的位置识别和控制。

## 主要功能

### 🎯 核心识别模块
- **四点/十二点检测**: 支持识别矿物兑换台的四个关键点位和十二个关键点位
- **V字标记识别**: 检测侧面V字形标记，用于精确定位
- **ArUco标记定位**: 支持ArUco标记检测用于精确定位机械臂位置
- **深度信息融合**: CUDA实现结合RGB和深度信息进行三维定位
- **双定位方案**: 单目RealSense定位和单目（大恒工业相机）+RealSense联合定位


### 🛠️ 技术特性
- **多相机支持**: 支持RealSense深度相机和大恒相机
- **实时处理**: 多线程处理架构，保证实时性能
- **参数配置**: 灵活的JSON配置系统
- **调试模式**: 完整的调试和可视化功能

## 系统架构

```
TJURM-Engineering/
├── src/
│   ├── main.cpp              # 主程序入口
│   ├── aruco/                # ArUco标记检测模块
│   ├── cuda/                 # CUDA加速模块
│   ├── data_manager/         # 数据管理和参数控制
│   ├── locate/               # 定位算法模块
│   └── mining_tank/          # 矿物兑换台检测核心
├── include/                  # 头文件
├── config/                   # 配置文件
├── libs/                     # 第三方库
└── build/                    # 构建输出
```

## 依赖库

- **OpenCV 4.5.4** - 计算机视觉库
- **RealSense SDK** - 深度相机支持
- **OpenRM** - RoboMaster视觉库
- **Eigen3** - 线性代数库
- **Ceres** - 非线性优化库
- **CUDA** - GPU加速计算
- **TensorRT** - 深度学习推理加速
- **nlohmann/json** - JSON解析库

## 编译和运行

### 环境要求
- Ubuntu 18.04/20.04
- CMake 3.12+
- GCC 7.0+
- CUDA 11.0+
- RealSense SDK 2.0+

### 编译步骤

```bash
# 克隆项目
git clone <repository-url>
cd TJURM-Engineering

# 创建构建目录
mkdir build && cd build

# 配置CMake
cmake ..

# 编译
make -j4

# 运行程序
./TJURM-Engineering
```

## 配置说明

### 参数配置 (`config/param.json`)
- **Point**: 世界坐标系中的关键点位置
- **Debug**: 调试模式和显示选项
- **Camera**: 相机参数和标定信息
- **Threshold**: 图像处理阈值参数

### 相机标定 (`config/CamLens.json`)
- 相机内参矩阵
- 畸变系数
- 相机分辨率参数

## 核心算法

### 四点检测算法
1. **图像预处理**: 颜色通道分离和二值化
2. **轮廓筛选**: 基于面积比、长宽比等特征筛选
3. **几何拟合**: 多边形拟合和角点检测
4. **空间排序**: 按顺时针方向排序四个角点

### V字标记识别
1. **轮廓检测**: 检测V字形轮廓
2. **形状分析**: 基于几何特征筛选V字形状
3. **方向判断**: 确定V字的开口方向
4. **3D定位**: 结合深度信息计算空间位置

### 定位算法
- **PnP求解**: 使用OpenCV的solvePnP进行姿态估计
- **深度信息校验**: 利用深度信息计算得到的平面度、角度、长度校验检测结果
- **深度融合**: 结合RGB和深度信息提高精度
- **坐标变换**: 从相机坐标系到世界坐标系转换

## 调试功能

系统提供多种调试模式：
- `show_image_flag`: 显示处理后的图像
- `show_binary_image_flag`: 显示二值化图像
- `show_contour_flag`: 显示轮廓检测结果
- `show_triangle_flag`: 显示关键点检测结果
- `show_depth`: 显示深度图像

## 性能优化

- **多线程处理**: 图像获取和处理分离
- **CUDA加速**: 利用GPU加速图像处理
- **内存管理**: 智能指针和内存池优化
- **实时调度**: 线程优先级设置

## 贡献指南

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

## 版权信息

本项目为天津大学北洋机甲工程视觉系统，用于RoboMaster比赛。

## 联系方式

- **团队**: 天津大学北洋机甲 (TJURM)
- **比赛**: RoboMaster 2025
- **类别**: 工程视觉系统
