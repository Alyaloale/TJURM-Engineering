#include "data_manager/control/control.h"

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>

using namespace rm;


void Control::autodetect() {

    if (Data::serial_flag) {
        std::thread send_thread(&Control::send_thread, this);
        setThreadPriority(send_thread, SCHED_RR, 97);
        send_thread.detach();
    }

    std::thread detect_start(&Control::detect_start, this);
    setThreadPriority(detect_start, SCHED_RR, 98);
    detect_start.detach();

    // std::thread aruco_detect_thread(&Control::aruco_detect, this);
    // setThreadPriority(aruco_detect_thread, SCHED_RR, 98);
    // aruco_detect_thread.detach();

    if (Data::serial_flag) {
        std::thread receive_thread(&Control::receive_thread, this);
        setThreadPriority(receive_thread, SCHED_RR, 96);
        receive_thread.detach();
    }
}
