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

#define SOF 0xA5

class Control {
public:
    static std::shared_ptr<Control> get_instance() {
        static std::shared_ptr<Control> instance(new Control());
        return instance;
    }


    rm::ArmorColor get_self() {
        rm::ArmorColor enemy_color = static_cast<rm::ArmorColor>(this->state_bytes_.input_data.enemy_color);
        if (enemy_color == rm::ARMOR_COLOR_BLUE) return rm::ARMOR_COLOR_RED;
        else if (enemy_color == rm::ARMOR_COLOR_RED) return rm::ARMOR_COLOR_BLUE;
    }


    void send_thread();
    void receive_thread();
    void autodetect();
    void message();
    void state();

    void send_single(double yaw, double pitch, bool fire, rm::ArmorID id = rm::ARMOR_ID_UNKNOWN);
    void stop_send() { send_flag_ = false; }
    void start_send() {
        send_flag_ = true;
        send_cv_.notify_all();
    }


private:
    Control() {}
    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;


public:
    std::deque<std::pair<TimePoint, StateBytes>> state_queue_;

    StateBytes              state_bytes_;             // 电控 -> 自瞄
    OperateBytes            operate_bytes_;           // 自瞄 -> 电控

    int                     file_descriptor_;
    std::string             port_name_;

private:
    bool                    send_flag_ = true;
    std::condition_variable send_cv_;
};

#endif