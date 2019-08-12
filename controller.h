#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Period of controller
#define PERIOD 0.2

// Deadzone values
#define LEFT_DEADZONE_POS  1.8942
#define RIGHT_DEADZONE_POS 1.8517

#include <stdint.h>
#include "index.hpp"

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
typedef struct thread_args{
    volatile uint8_t* arg_pwms;
    volatile double* arg_refs;
    volatile double* arg_spds;
} controlArgs;

void* speed_control(void *args);

#endif