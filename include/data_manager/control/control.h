#ifndef TJURM_AIMING_THREADS_SERIAL_H_
#define TJURM_AIMING_THREADS_SERIAL_H_
#include <mutex>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include "data_manager/base.h"
#include "data_manager/param.h"
#include "data_manager/control/structure.h"

#include <thread>
#include <pthread.h>

void setThreadPriority(std::thread& thread, int policy, int priority);

SharedData* init_shared_memory();// 初始化共享内存

void update_shared_data(SharedData* data, const float new_matrix[4][4], short new_color);// 更新共享内存

void read_shared_data(SharedData* data, float out_matrix[4][4], short* out_color, uint64_t* out_ts);// 读取共享内存


class Control {
public:
    static std::shared_ptr<Control> get_instance() {
        static std::shared_ptr<Control> instance(new Control());
        return instance;
    }

    void autodetect();
    void detect_start();
    void aruco_detect();



private:
    Control() {}
    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;
};

#endif