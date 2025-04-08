#ifndef CONTROL_STRUCTURE_H_
#define CONTROL_STRUCTURE_H_
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>


struct SharedData{
    pthread_mutex_t mutex;       // 互斥锁确保独占访问
    double matrix[4][4];          // 4x4变换矩阵
    short color;                 // 我方颜色
    uint64_t version;            // 版本号
};

// 共享内存名称
SharedData* init_shared_memory();// 初始化共享内存

void update_shared_data(
    SharedData* data,
    const double new_matrix[4][4]  // 参数类型改为 double
); // 原子化写入（统一矩阵类型为 double）

void read_shared_data(
    SharedData* data,
    short* out_color
); // 原子化读取（完整数据拷贝）

void destroy_shared_memory(SharedData* data); // 销毁共享内存
#endif