#include "controller.h"

using namespace std;

static uint8_t get_pwm_from_voltage(double x){
    double input_voltage = rc_adc_dc_jack();

    int value = 255*(x/input_voltage);

    return (value > 255 ? 255 : uint8_t(value));
}

static void motor_set_voltage(int motors, double voltage, volatile uint8_t *pwm_to_send){
    if(motors & MOTOR_LEFT)
        pwm_to_send[0] = get_pwm_from_voltage(voltage);
    
    if(motors & MOTOR_RIGHT)
        pwm_to_send[1] = get_pwm_from_voltage(voltage);
}

static double anti_dz(double voltage, double deadzone_p, double deadzone_m = 0){
    // Use threshold as 0.001
    if(voltage > 1e-3)
        voltage += deadzone_p;
    else if(voltage < -1e-3)
        voltage += deadzone_m;

    return voltage;
}

static double saturate(double voltage){
    if(voltage > MAXIMUM_VOLTAGE)
        voltage = MAXIMUM_VOLTAGE;
    else if(voltage < MINIMUM_VOLTAGE)
        voltage = MINIMUM_VOLTAGE;

    return voltage;
}

void* speed_control(void *args){

    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Values to be sent to motor
    double left_voltage = 0, right_voltage = 0;
    
    // Castings
    controlArgs* control_arguments = (controlArgs*)args;
    volatile uint8_t* pwms = control_arguments->arg_pwms;
    volatile double* refs = control_arguments->arg_refs;
    volatile double* readings = control_arguments->arg_spds;

    // Sync
    mutex* sensors_mutex = control_arguments->arg_sensors_mutex;
    condition_variable* sensors_cv = control_arguments->arg_sensors_cv;

    // Round arrays for control calculations
    double err_left[3] = {0, 0, 0},
           err_right[3] = {0, 0, 0},
           u_left[3] = {0, 0, 0},
           u_right[3] = {0, 0, 0};

    #if (USING_ENCODER == 0)
    // Variables used because the encoder readings are not working properly
    double final_speed[2], actual_speed[2] = {0.0, 0.0};
    #endif

    // Special kind of index
    index control_idx(3);

    // Sync with sensors thread
    unique_lock<mutex> sensors_lock(*sensors_mutex);
    sensors_cv->wait(sensors_lock);
    sensors_lock.unlock();

    // TODO: Insert control with angle error, refs[1]

    for(;;){

        // If encoders readings cannot be trusted
        #if USING_ENCODER
            // Update error with base on readings
            sensors_lock.lock();

            err_left[control_idx.idx()] = refs[0] - readings[INDEX_LEFT];
            err_right[control_idx.idx()] = refs[0] - readings[MOTOR_RIGHT];

            sensors_lock.unlock();
        #else
            // Update error based on simulated speed
            
            final_speed[INDEX_LEFT] = K_LEFT*u_left[control_idx.idx(-1)];

            // If it's the first measure from last change of output
            if( abs(u_left[control_idx.idx(-1)] - u_left[control_idx.idx(-2)]) < 1e-4 ){
                actual_speed[INDEX_LEFT] = final_speed[INDEX_LEFT] + (actual_speed[INDEX_LEFT] - final_speed[INDEX_LEFT])*exp(-CONTROLLER_PERIOD/TAU_LEFT);
            }
            else {
                actual_speed[INDEX_LEFT] = final_speed[INDEX_LEFT];
            };
            err_left[control_idx.idx()] = refs[0] - actual_speed[INDEX_LEFT];

            // Right side
            final_speed[INDEX_RIGHT] = K_DIR*u_right[control_idx.idx(-1)];
            
            // If it's the first measure
            if( abs(u_right[control_idx.idx(-1)] - u_right[control_idx.idx(-2)]) < 1e-4 ){
                actual_speed[INDEX_RIGHT] = final_speed[INDEX_RIGHT] + (actual_speed[INDEX_RIGHT] - final_speed[INDEX_RIGHT])*exp(-CONTROLLER_PERIOD/TAU_DIR);
            }
            else {
                actual_speed[INDEX_RIGHT] = final_speed[INDEX_RIGHT];
            };

            err_right[control_idx.idx()] = refs[0] - actual_speed[INDEX_RIGHT];
        #endif

        /*--------------------*/
        // Left motor control //
        /*--------------------*/

        //   Control signal calculation
        u_left[control_idx.idx()] = 0.4806*err_left[control_idx.idx(-2)] + 0.4*u_left[control_idx.idx(-1)] + 0.6*u_left[control_idx.idx(-2)];

        //   Deadzone compensation
        left_voltage = anti_dz(u_left[control_idx.idx()], LEFT_DEADZONE_POS);

        //   Saturation
        left_voltage = saturate(left_voltage);

        /*---------------------*/
        // Right motor control //
        /*---------------------*/

        //   Control signal calculation
        u_right[control_idx.idx()] = 0.4373*err_right[control_idx.idx(-2)] + 0.5075*u_right[control_idx.idx(-1)] + 0.4925*u_right[control_idx.idx(-2)];

        //   Deadzone compensation
        right_voltage = anti_dz(u_right[control_idx.idx()], RIGHT_DEADZONE_POS);

        //   Saturation
        right_voltage = saturate(right_voltage);

        // Update index for circular updating variables
        control_idx++;

        // Update PWM to be sent to arduino
        motor_set_voltage(MOTOR_LEFT, left_voltage, pwms);
        motor_set_voltage(MOTOR_RIGHT, right_voltage, pwms);

        // Ugly simulation of controller period
        rc_usleep(CONTROLLER_PERIOD*1e6);
        
        if(rc_get_state() == EXITING)
            break;
    }

    return NULL;
}