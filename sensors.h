#ifndef SENSORS_H_
#define SENSORS_H_

#define DIAMETER 0.121 // meters
#define CPR 1632.67
#define PI 3.14159

#define COUNT_GYRO 200

// Factor to account for differences in the terrain which affect difference between simulated and real speed
#define TERRAIN_FACTOR 2.5

#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include "index.h"
#include "main.h"
#include "controller.h" // For PERIOD

extern "C" {
    #include <rc/start_stop.h>
    #include <rc/time.h>
    #include <rc/encoder.h>
    #include <rc/encoder_eqep.h>
    #include <rc/mpu.h>
}

using namespace std;

// Arguments for sensors thread:
//      arg_readings -> 4 array of double:
//                          [0] -> Total distance walked
//                          [1] -> Total angle displacement
//                          [2] -> Speed of left motor
//                          [3] -> Speed of right motor                              
//      arg_control_mutex -> Mutex to sync with control thread
//      arg_control_cv -> Sync with control thread
//      arg_nagivation_mutex -> Mutex to avoid problems with navigation thread
typedef struct sensors_thread_args{
    double* arg_readings;
    mutex* arg_control_mutex;
    condition_variable* arg_control_cv;
    mutex* arg_navigation_mutex;
}sensorsArgs;

void* filter_sensors(void *arg);

#endif