#include "data_manager/control/control.h"
#include "data_manager/param.h"

#include <mutex>
#include <condition_variable>


void setThreadPriority(std::thread& thread, int policy, int priority) {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, policy);

    sched_param param;
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    pthread_setschedparam(thread.native_handle(), policy, &param);
}
