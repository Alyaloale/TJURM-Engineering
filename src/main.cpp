#include "data_manager/base.h"
#include "data_manager/param.h"
#include "mining_tank/detector.h"
#include "data_manager/control/control.h"

std::mutex hang_up_mutex;
std::condition_variable hang_up_cv;
using namespace rm;


int main(int argc, char** argv) {
    auto control = Control::get_instance();
    init_debug();
    if(!Data::debug) while(true) if(init_camera()) break;
    if (Data::serial_flag) init_serial();

    control->autodetect();

    rm::message("Main thread hang up!", rm::MSG_OK);
    std::unique_lock<std::mutex> lock(hang_up_mutex);
    hang_up_cv.wait(lock);
    return 0;
}