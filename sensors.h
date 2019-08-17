#ifndef SENSORS_H_
#define SENSORS_H_

#define DIAMETER 0.121 // meters
#define CPR 1632.67
#define PI 3.14159

#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "index.h"

extern "C" {
    #include <rc/start_stop.h>
    #include <rc/time.h>
    #include <rc/encoder.h>
    #include <rc/encoder_eqep.h>
    #include <rc/mpu.h>
}

using namespace std;

// Parameter: double array with 4 positions:
//      0 -> Total distance walked
//      1 -> Total angle displacement
//      2 -> Speed of left motor
//      3 -> Speed of right motor

void* filter_sensors(void *arg);

#endif