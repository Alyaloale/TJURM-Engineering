#ifndef TJURM_AIMING_THREADS_SERIAL_H_
#define TJURM_AIMING_THREADS_SERIAL_H_
#include <mutex>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <thread>
#include "data_manager/base.h"
#include "data_manager/param.h"
#include "data_manager/control/structure.h"

void setThreadPriority(std::thread& thread, int policy, int priority);


class Control {
public:
    static std::shared_ptr<Control> get_instance() {
        static std::shared_ptr<Control> instance(new Control());
        return instance;
    }

    void autodetect();
    void detect_start();
    //void aruco_detect();



private:
    Control() {}
    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;
};

#endif