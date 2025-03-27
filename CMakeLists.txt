cmake_minimum_required(VERSION 3.12)
project(TJURM-Engineering)

# 设置编译指令
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

# 设置目标兵种
add_definitions(-DTJURM_Engineering)

# CUDA
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)

#TensorRT
include_directories(/home/tjurm/Downloads/TensorRT-8.4.3.1/include)
link_directories(/home/tjurm/Downloads/TensorRT-8.4.3.1/lib)

# OpenRM
find_package(OpenRM REQUIRED)
include_directories(${OpenRM_INCLUDE_DIRS})


# OpenCV
find_package(OpenCV 4.5.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#Realsense
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIR})

# 添加项目头文件
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/json/single_include/nlohmann)

# 添加构建子目录
add_subdirectory(src/data_manager)
add_subdirectory(src/mining_tank)
add_subdirectory(src/aruco)
add_subdirectory(src/locate)
add_subdirectory(src/cuda)

# 添加可执行文件
add_executable(TJURM-Engineering src/main.cpp)

# 链接模块静态库
target_link_libraries(TJURM-Engineering
    data_manager
    mining_tank
    aruco
    locate
    cudatools
)

