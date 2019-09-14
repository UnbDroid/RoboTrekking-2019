#include <iostream>
#include <stdint.h>
#include <mutex>
#include <condition_variable>
#include "communication.h"
#include "sensors.h"
#include "main.h"

#if 1
    #include "controller.h"
#else
    #define MAIN_ID
    #include "identification.h"
#endif

using namespace std;

extern "C" {
    #include <rc/start_stop.h>
    #include <rc/encoder.h>
    #include <rc/encoder_eqep.h>
    #include <rc/time.h>
    #include <rc/pthread.h>
}

int main(){
    cout << "Lembrou de usar config-pin em todos os pinos?" << endl;

    /* Shared variables */

    // Shared between low-level control and communication threads, holds the pwm value to be send to arduino
    volatile uint8_t pwm_to_send[2] = {0, 0};

    // Shared between low and high-level control threads, hold speed and angle error
    volatile double references[2] = {1.5, 0};

    // Shared between low and high-level control threads and reading sensors thread, holds:
    //      0 -> Total distance walked
    //      1 -> Total angle displacement
    //      2 -> Speed of left motor
    //      3 -> Speed of right motor
    volatile double general_readings[4] = {0.0, 0.0, 0.0, 0.0};

    /* Variables for synchronization */
    mutex control_sensors_mutex;
    condition_variable control_sensors_cv;

    if(rc_enable_signal_handler() == -1){
        return -1;
    }

    // Threads
    pthread_t comm_thread;
    pthread_t control_thread;
    pthread_t sensors_thread;

    // initialize 3 main encoders, avoiding problems with PRU
	if(rc_encoder_eqep_init()){
		return -1;
	}

    // Start voltage ADC reader
    if(rc_adc_init()==-1) 
        return -1;

    // Starts thread that sends info to the arduino, SCHED_OTHER is the common RR
    if( rc_pthread_create(&comm_thread, send_pwm, (void*)pwm_to_send, SCHED_FIFO, 1) != 0){
        cout << "Could not start communication thread!" << endl;
    }

    #ifdef MAIN_ID
        // Use the same thread variable for simplicity
        rc_pthread_create(&control_thread, generate_id_data, (void*)pwm_to_send, SCHED_FIFO, 2);
    #else    
        // Arguments for low-level control thread
        controlArgs control_args;
        control_args.arg_pwms = (uint8_t*)pwm_to_send;
        control_args.arg_refs = (double*)references;
        control_args.arg_spds = (double*)&general_readings[2];
        control_args.arg_sensors_mutex = &control_sensors_mutex;
        control_args.arg_sensors_cv = &control_sensors_cv;

        // Starts thread that controls the speed of the motors
        rc_pthread_create(&control_thread, speed_control, (void*) &control_args, SCHED_FIFO, 3);

        // Arguments for sensors thread
        sensorsArgs sensors_args;
        sensors_args.arg_readings = (double*)general_readings;
        sensors_args.arg_control_mutex = &control_sensors_mutex;
        sensors_args.arg_control_cv = &control_sensors_cv;

        // Sensors thread initialization
        if( rc_pthread_create(&sensors_thread, filter_sensors, (void*)&sensors_args, SCHED_FIFO, 2) != 0){
            cout << "Could not start sensors thread!" << endl;
        }
    #endif

    for(;;){
        // Infinite loop to get ctrl-C and exit program
        // Allows threads to run indefinitly

        if(rc_get_state() == EXITING)
            break;
    }

    // The exiting is commanded by another thread

    rc_set_state(EXITING);

    rc_encoder_eqep_cleanup();

    rc_adc_cleanup();
}
