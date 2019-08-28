#ifndef SENSORS_H_
#define SENSORS_H_

#define DIAMETER 0.121 // meters
#define CPR 1632.67
#define PI 3.14159

#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include "index.h"
#include "main.h"

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
typedef struct sensors_thread_args{
    double* arg_readings;
    mutex* arg_control_mutex;
    condition_variable* arg_control_cv;
}sensorsArgs;

void* filter_sensors(void *arg);

#endif