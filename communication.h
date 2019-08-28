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

// Receives address of array containing 2 pwm values to be sent to the motors
void* send_pwm(void *pwm);

#endif