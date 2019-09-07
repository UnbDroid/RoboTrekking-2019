#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Period of controller
#define CONTROLLER_PERIOD 0.2

// Deadzone values
#define LEFT_DEADZONE_POS  1.8942
#define RIGHT_DEADZONE_POS 1.4517

// Left motor constants
#define K_LEFT 0.6139
#define TAU_LEFT 0.1324

// Right motor constants
#define K_DIR 0.6068
#define TAU_DIR 0.1334

#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include "main.h"
#include "index.h"

using namespace std;

extern "C" {
    #include <rc/time.h>
    #include <rc/adc.h>
    #include <rc/start_stop.h>
}

// Arguments for low-level control thread:
//      arg_pwms -> Pwm of speed to send to motors
//      arg_refs -> Desired speed (m/s) at index 0 and angle error (degrees) at index 1
//      arg_spds -> Measured speed of 0 -> left motor, 1 -> right motor
//      arg_sensor_mutex -> Mutex to sync with sensors thread
//      arg_sensor_cv -> Condition variable for sync
//      arg_refs_mutex -> Mutex to sync references signals
typedef struct control_thread_args{
    uint8_t* arg_pwms;
    double* arg_refs;
    double* arg_spds;
    mutex* arg_sensors_mutex;
    condition_variable* arg_sensors_cv;
    mutex* arg_refs_mutex;
} controlArgs;

void* speed_control(void *args);

#endif