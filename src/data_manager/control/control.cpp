#include "data_manager/control/control.h"
#include "data_manager/control/crc.h"

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>

using namespace rm;

void Control::send_single(double yaw, double pitch, bool fire, rm::ArmorID id) {
    if (!Data::serial_flag) return;

    operate_bytes_.frame_header.sof = SOF;
    //operate_bytes_.output_data.shoot_yaw = static_cast<float>(yaw);
    //operate_bytes_.output_data.shoot_pitch = static_cast<float>(pitch);
    //operate_bytes_.output_data.fire = fire;


    append_crc16_check_sum((uint8_t*)&operate_bytes_, sizeof(OperateBytes));
    int status = (int)rm::writeToSerialPort((uint8_t*)&operate_bytes_, sizeof(operate_bytes_), file_descriptor_);
    if (status) {
        rm::message("Control error: " + std::to_string(status), rm::MSG_ERROR);
        if (access(port_name_.c_str(), F_OK) < 0) {
            init_serial();
            status = 0;
        } else {
            status = (int)rm::restartSerialPort(file_descriptor_, port_name_);
        }
    }
}
void Control::autodetect() {
    std::thread send_thread(&Control::send_thread, this);
    setThreadPriority(send_thread, SCHED_RR, 97);
    send_thread.detach();

    // std::thread detect_start(&Control::detect_start, this);
    // setThreadPriority(detect_start, SCHED_RR, 98);
    // detect_start.detach();

    std::thread aruco_detect_thread(&Control::aruco_detect, this);
    setThreadPriority(aruco_detect_thread, SCHED_RR, 98);
    aruco_detect_thread.detach();

    if (Data::serial_flag) {
        std::thread receive_thread(&Control::receive_thread, this);
        setThreadPriority(receive_thread, SCHED_RR, 96);
        receive_thread.detach();
    }
}
