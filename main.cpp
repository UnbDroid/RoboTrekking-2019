#define ENCODER_L 1
#define ENCODER_R 2

#define MOTOR_LEFT 1<<0
#define MOTOR_RIGHT 1<<1

#define MAXIMUM_VOLTAGE 7
#define MINIMUM_VOLTAGE 0
#define SAMPLES 1000
#define N_PERIODS 1

// Period of sampling in seconds
#define PERIOD 0.02

#include <cstdio>
#include <iostream>
#include <fstream>

using namespace std;

extern "C" {
    #include <rc/encoder.h>
    #include <rc/time.h>
    #include <rc/adc.h>
}

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
    
        out1 = rc_encoder_read(ENCODER_L);
        out2 = rc_encoder_read(ENCODER_R);

        input[i] = value;
        outputs[i] = make_pair(out1, out2);

        value = value + rate;

        cout << value << endl;

        // Ugly simulation of period
        rc_usleep(int(PERIOD*1e6));
    }
}

void save_to_file(){
    uint i = 0;
    
    ofstream data_file;
    data_file.open("data.txt");

    for(;i<SAMPLES;i++){
        data_file << input[i] << " " << outputs[i].first << " " << outputs[i].second << endl;
    }

    data_file.close();
}

int main(){
    cout << "Lembrou de usar pin-config em todos os pinos?" << endl;

    // initialize all encoders, maybe we can initialize just the ones we are using?
	if(rc_encoder_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_init\n");
		return -1;
	}

    // Start voltage ADC reader
    if(rc_adc_init()==-1) return -1;

    // Starts thread that sends info to the arduino
    // Here goes the thread that calls the functions

    send_triangular_wave();

    save_to_file();
}