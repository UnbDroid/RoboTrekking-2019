#ifndef NAVIGATION_H_
#define NAVIGATION_H_

// Period of navigation
#define PERIOD 0.2

// Speed (m/s)
#define MAX_SPEED       3
#define APROX_SPEED     2
#define CIRCLE_SPEED    1
#define STOP            0

#include <stdint.h>
#include <math.h>
#include <cstdlib>

extern "C" {
    #include <rc/time.h>
    #include <rc/adc.h>
    #include <rc/start_stop.h>
}

// States of the navigation
enum State{
    START,
    GO_TO_FIRST,
    GO_TO_SECOND,
    GO_TO_LAST,
    GO_ROUND,
    DODGE,
    END
};

typedef struct thread_args{
    volatile double* arg_refs;
    volatile double* arg_g_readings;
} navigationArgs;

// Args are the references shared between low-level controller and high-level controller
void* navigation_control(void* args);

#endif