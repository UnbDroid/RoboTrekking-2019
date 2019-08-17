#include "identification.h"

static double ts[SAMPLES];
static double input[SAMPLES];
static pair<double, double> outputs[SAMPLES];

static uint8_t get_pwm_from_voltage(double x){
    double input_voltage = rc_adc_dc_jack();

    int value = 255*(x/input_voltage);

    return (value > 255 ? 255 : uint8_t(value));
}

static void motor_set_voltage(int motors, double voltage, uint8_t *pwm_to_send){
    if(motors & MOTOR_LEFT)
        pwm_to_send[0] = get_pwm_from_voltage(voltage);
    
    if(motors & MOTOR_RIGHT)
        pwm_to_send[1] = get_pwm_from_voltage(voltage);
}

#ifdef ID_TRIANGULAR
static void send_triangular_wave(uint8_t* pwms){
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

        motor_set_voltage(MOTOR_LEFT|MOTOR_RIGHT, value, pwms);

        // Sleep for some time so the communication thread can send the signals
        rc_nanosleep(1e5);

        out1 = rc_encoder_eqep_read(ENCODER_L);
        out2 = rc_encoder_eqep_read(ENCODER_R);

	    ts[i] = time;
        input[i] = value;
        outputs[i] = make_pair(out1, out2);

        value = value + rate;

        // Ugly simulation of period
        rc_nanosleep(int(PERIOD*1e9));
        
        temp = rc_nanos_since_boot();
        time += (temp-nanos_since_boot)/(double)1e9;
        nanos_since_boot = temp;

        if(rc_get_state() == EXITING)
            break;
    }
}
#endif

#ifdef ID_SQUARE
static void send_square_wave(uint8_t *pwms){
    int i=0, out1, out2;
    double value, time=0;

    uint8_t state = 0;

    uint32_t samples_per_cycle = SAMPLES/N_PERIODS;

    uint64_t nanos_since_boot = rc_nanos_since_boot(), temp;

    // Starts low and each half-cycle change it's state
    for(;i<SAMPLES;i++){
        if(state){
            value = MAXIMUM_VOLTAGE;
        }
        else{
            value = MINIMUM_VOLTAGE;
        }

        motor_set_voltage(MOTOR_LEFT|MOTOR_RIGHT, value, pwms);

        out1 = rc_encoder_eqep_read(ENCODER_L);
        out2 = rc_encoder_eqep_read(ENCODER_R);

	    ts[i] = time;
        input[i] = value;
        outputs[i] = make_pair(out1, out2);

        if(i%(samples_per_cycle/2) == 0){
            state = (state+1)%2;
        }

        // Ugly simulation of period
        rc_usleep(int(PERIOD*1e6));
        
        temp = rc_nanos_since_boot();
        time += (temp-nanos_since_boot)/(double)1e9;
        nanos_since_boot = temp;

        if(rc_get_state() == EXITING)
            break;
    }
}
#endif

static void save_to_file(){
    uint i = 0;
    
    ofstream data_file;
    data_file.open("data.txt");

    for(;i<SAMPLES;i++){
        data_file << ts[i] << "," << input[i] << "," << outputs[i].first << "," << outputs[i].second << endl;
    }

    data_file.close();
}

void* generate_id_data(void *arg){

    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Casting
    uint8_t* pwms = (uint8_t*)arg;

    #ifdef ID_TRIANGULAR
        send_triangular_wave(pwms);
    #elif defined ID_SQUARE
        send_square_wave(pwms);
    #endif

    save_to_file();

    rc_set_state(EXITING);

    return NULL;
}