#include "data_manager/control/control.h"
#include <thread>
#include <cmath>
#include <fstream>
#include "mining_tank/detector.h"
using namespace rm;


void Control::message() {
    //rm::message_send();
    rm::message("enemy color", Data::mining_tank_color);
    rm::message("camera id", Data::camera_index);
}


void Control::send_thread() {

    std::mutex mutex;
    TimePoint trigger_control = getTime();
    while(true) {
        if(!send_flag_) {
            std::unique_lock<std::mutex> lock(mutex);
            send_cv_.wait(lock, [this]{return send_flag_;});
        }
        std::this_thread::sleep_for(std::chrono::microseconds(Data::send_wait_time_ms));
        this->message();            // 统一终端发消息
    }
}