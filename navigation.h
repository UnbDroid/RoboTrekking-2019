#ifndef NAVIGATION_H_
#define NAVIGATION_H_

// Period of navigation
#define NAVIGATION_PERIOD 0.2

#define DISTANCE_TO_START_GO_AROUND 0.7 //meters
#define DISTANCE_TO_USE_VISION 2.0 //meters
#define OFFSET_ANGLE_TO_START_CIRCLE 20.0
#define ANGLE_TO_GO_AROUND 90.0
#define DISTANCE_TO_GO_AROUND 1.0 //meters

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
    GO_AROUND,
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