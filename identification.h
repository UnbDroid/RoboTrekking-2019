#ifndef IDENTIFICATION_H_
#define IDENTIFICATION_H_

#define SAMPLES 10000
#define N_PERIODS 4

#include "main.h"

// Choose between triangular or square identification signal
#if 1
    #define ID_TRIANGULAR
#else
    #define ID_SQUARE
#endif

// Period of sampling in seconds
#define PERIOD 0.002

#include <fstream>
#include <stdint.h>

extern "C" {
    #include <rc/encoder.h>
    #include <rc/encoder_eqep.h>
    #include <rc/time.h>
    #include <rc/adc.h>
    #include <rc/start_stop.h>
}

using namespace std;

void* generate_id_data(void *arg);

#endif