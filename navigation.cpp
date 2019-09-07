#include "navigation.h"
#include "vision.h"

void* navigation_control(void* args){
    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Castings
    navigationArgs* navigation_arguments = (navigationArgs*)args;
    volatile double* refs = navigation_arguments->arg_refs;
    volatile double* readings = navigation_arguments->arg_g_readings;

    // Sync
    mutex* refs_mutex = navigation_arguments->arg_refs_mutex;
    mutex* sensors_mutex = navigation_arguments->arg_sensors_mutex;

    unique_lock<mutex> refs_lock(*refs_mutex, defer_lock);
    unique_lock<mutex> sensors_lock(*sensors_mutex, defer_lock);

    // Set reference
    sensors_lock.lock();
    refs_lock.lock();

    refs[0] = 1.5;

    refs_lock.unlock();
    sensors_lock.unlock();

    for(;;){

        // Safely update angle error
        sensors_lock.lock();
        refs_lock.lock();

        refs[1] = -readings[1];

        refs_lock.unlock();
        sensors_lock.unlock();

        rc_usleep(NAVIGATION_PERIOD*1e6); // 1s = 1e6 useconds

        if(rc_get_state() == EXITING)
            break;
    }

    return NULL;
}