#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <stdint.h>
#include <cstring>
#include "main.h"

using namespace std;

extern "C"{
    #include <rc/uart.h>
    #include <rc/start_stop.h>
}

#define TIMEOUT_S 0.5
#define BAUDRATE 115200

typedef struct comm_thread_args {
    uint8_t* arg_pwms;
    bool* which_us;
    bool* flag;
} commArgs;

// Receives address of array containing 2 pwm values to be sent to the motors
void* send_pwm(void* args);

#endif