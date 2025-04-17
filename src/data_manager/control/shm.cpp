#include "data_manager/control/control.h"
#include <thread>
#include <cmath>
#include <fstream>
#include "mining_tank/detector.h"


#define SHM_NAME "/transform_matrix_shm"
#define SHM_SIZE sizeof(SharedData)


/* 初始化共享内存（生产者端） */
SharedData* init_shared_memory() {
    // 创建共享内存对象（O_EXCL 确保唯一创建者）
    int fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0777);
    if (fd == -1) {
        if (errno == EEXIST) {
            // 共享内存已存在，需先清理
            shm_unlink(SHM_NAME);
            fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0777);
        } else {
            perror("shm_open failed");
            return NULL;
        }
    }
    ftruncate(fd, SHM_SIZE);

    SharedData* data = (SharedData*)mmap(NULL, SHM_SIZE, 
                           PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);  // mmap 后文件描述符可立即关闭

    // 初始化进程间互斥锁属性
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&data->mutex, &mutex_attr);
    pthread_mutexattr_destroy(&mutex_attr);  // 修正：销毁属性

    // 初始化默认值
    memset(data->matrix, 0, sizeof(double[4][4]));
    data->color = 0;
    data->version = 0;

    //更改内存权限
    int ret = system("chmod 777 /dev/shm/transform_matrix_shm");
    if (ret != 0) {
        perror("Permission denied");
    }
    return data;
}

/* 原子化写入（统一矩阵类型为 double） */
void update_shared_data(
    SharedData* data,
    const double new_matrix[4][4]  // 参数类型改为 double
) {
    pthread_mutex_lock(&data->mutex);
    memcpy(data->matrix, new_matrix, sizeof(double[4][4]));  // 安全复制
    data->version++;
    pthread_mutex_unlock(&data->mutex);
}

/* 原子化读取（完整数据拷贝） */
void read_shared_data(
    SharedData* data,
    short* out_color
) {
    pthread_mutex_lock(&data->mutex);
    *out_color = data->color;
    pthread_mutex_unlock(&data->mutex);
}

/* 清理共享内存资源 */
void cleanup_shared_memory(SharedData* data) {
    munmap(data, SHM_SIZE);
    shm_unlink(SHM_NAME);  // 由创建者调用
}