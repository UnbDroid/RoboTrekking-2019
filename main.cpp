#define ENCODER_L 2
#define ENCODER_R 1

#define MOTOR_LEFT 1<<0
#define MOTOR_RIGHT 1<<1

#define MAXIMUM_VOLTAGE 7
#define MINIMUM_VOLTAGE 0
#define SAMPLES 500
#define N_PERIODS 3

// Period of sampling in seconds
#define PERIOD 0.02

#include <cstdio>
#include <iostream>
#include <fstream>
#include "communication.hpp"

using namespace std;

extern "C" {
    #include <rc/encoder.h>
    #include <rc/encoder_eqep.h>
    #include <rc/time.h>
    #include <rc/adc.h>
    #include <rc/pthread.h>
}

double ts[SAMPLES];
double input[SAMPLES];
pair<double, double> outputs[SAMPLES];

volatile uint8_t pwm_to_send[2];

uint8_t get_pwm_from_voltage(double x){
    double input_voltage = rc_adc_dc_jack();

    int value = 255*(x/input_voltage);

    return (value > 255 ? 255 : uint8_t(value));
}

void motor_set_voltage(int motors, double voltage){
    if(motors & MOTOR_LEFT)
        pwm_to_send[0] = get_pwm_from_voltage(voltage);
    
    if(motors & MOTOR_RIGHT)
        pwm_to_send[1] = get_pwm_from_voltage(voltage);
}

void send_triangular_wave(){
    int i=0, out1, out2;
    double value = MINIMUM_VOLTAGE;
    double time = 0;

    uint64_t nanos_since_boot = rc_nanos_since_boot(), temp;

    double rate = (MAXIMUM_VOLTAGE-MINIMUM_VOLTAGE)/double(SAMPLES/(N_PERIODS*2));

    for(;i<SAMPLES;i++){
        if((value >= MAXIMUM_VOLTAGE) && (rate > 0)){
            value = MAXIMUM_VOLTAGE;
            rate = -rate;
        }
        else if((value <= MINIMUM_VOLTAGE) && (rate < 0)){
            value = MINIMUM_VOLTAGE;
            rate = -rate;
        }

        motor_set_voltage(MOTOR_LEFT|MOTOR_RIGHT, value);
    
        out1 = rc_encoder_eqep_read(ENCODER_L);
        out2 = rc_encoder_eqep_read(ENCODER_R);

	    ts[i] = time;
        input[i] = value;
        outputs[i] = make_pair(out1, out2);

        value = value + rate;

        // Ugly simulation of period
        rc_usleep(int(PERIOD*1e6));
        
        temp = rc_nanos_since_boot();
        time += (temp-nanos_since_boot)/(double)1e9;
        nanos_since_boot = temp;

        if(rc_get_state() == EXITING)
            break;
    }
}

void save_to_file(){
    uint i = 0;
    
    ofstream data_file;
    data_file.open("data.txt");

    for(;i<SAMPLES;i++){
        data_file << ts[i] << "," << input[i] << "," << outputs[i].first << "," << outputs[i].second << endl;
    }

    data_file.close();
}

int main(){
    cout << "Lembrou de usar pin-config em todos os pinos?" << endl;

    if(rc_enable_signal_handler() == -1){
        return -1;
    }

    // Threads
    pthread_t comm_thread;

    // initialize 3 main encoders, avoiding problems with PRU
	if(rc_encoder_eqep_init()){
		return -1;
	}

    // Start voltage ADC reader
    if(rc_adc_init()==-1) 
        return -1;

    // Starts thread that sends info to the arduino, SCHED_OTHER is the common RR
    rc_pthread_create(&comm_thread, send_pwm, (void*)pwm_to_send, SCHED_OTHER, 0);

    send_triangular_wave();

    rc_set_state(EXITING);

    save_to_file();

    rc_encoder_eqep_cleanup();

    rc_adc_cleanup();
}
