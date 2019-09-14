#ifndef NAVIGATION_H_
#define NAVIGATION_H_

// Period of navigation
#define NAVIGATION_PERIOD 0.2

#define DISTANCE_TO_START_GO_AROUND     0.7 //meters
#define DISTANCE_TO_USE_VISION          2.0 //meters

#define OFFSET_ANGLE_TO_START_CIRCLE    20.0
#define ANGLE_TO_GO_AROUND              90.0
#define BIG_ANGLE_TO_DODGE              1.0 //degrees
#define SMALL_ANGLE_TO_DODGE            0.5 //degrees
#define ANGLE_TO_ADD                    30.0 //degrees

#define IDEAL_ACCURACY                  5.0

// Speed (m/s)
#define MAX_SPEED       3
#define APROX_SPEED     2
#define CIRCLE_SPEED    1
#define STOP            0

#include <stdint.h>
#include <math.h>
#include <cstdlib>
#include <mutex>
#include "sensors.h"

extern "C" {
    #include <rc/time.h>
    #include <rc/adc.h>
    #include <rc/start_stop.h>
}

using namespace std;

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
    double* arg_refs;
    double* arg_g_readings;
    mutex* arg_refs_mutex;
    mutex* arg_sensors_mutex;
    bool* arg_us;
} navigationArgs;

// Args are the references shared between low-level controller and high-level controller
void* navigation_control(void* args);

#endif