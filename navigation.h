#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#define NAVIGATION_PERIOD 0.2

#include <stdint.h>
#include <cstdlib>
#include <mutex>
#include "main.h"

using namespace std;

extern "C" {
    #include <rc/time.h>
    #include <rc/start_stop.h>
}

// Arguments for low-level control thread:
//      arg_refs -> Desired speed (m/s) at index 0 and angle error (degrees) at index 1
//      arg_g_readings -> Get distance and gyro reading
//      arg_refs_mutex -> Mutex to sync references signals
typedef struct navigation_thread_args{
    double* arg_refs;
    double* arg_g_readings;
    mutex* arg_refs_mutex;
    mutex* arg_sensors_mutex;
} navigationArgs;

// Args are the references shared between low-level controller and high-level controller
void* navigation_control(void* args);

#endif