#include "data_manager/control/control.h"

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>

using namespace rm;


void Control::autodetect() {
    
    std::thread detect_start(&Control::detect_start, this);
    setThreadPriority(detect_start, SCHED_RR, 98);
    detect_start.detach();

    // std::thread aruco_detect_thread(&Control::aruco_detect, this);
    // setThreadPriority(aruco_detect_thread, SCHED_RR, 98);
    // aruco_detect_thread.detach();

}
