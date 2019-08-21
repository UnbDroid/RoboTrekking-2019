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
    volatile uint8_t* pwms = (volatile uint8_t*)control_arguments->arg_pwms;
    volatile double* refs = (volatile double*)control_arguments->arg_refs;
    volatile double* readings = (volatile double*)control_arguments->arg_spds;

    // Round arrays
    double err_left[3] = {0, 0, 0},
           err_right[3] = {0, 0, 0},
           u_left[3] = {0, 0, 0},
           u_right[3] = {0, 0, 0};

    // Variables used because the encoder is not working properly
    double final_speed, actual_speed = 0;

    // Special kind of index
    index control_idx = index(3);

    // TODO: Insert control with angle error, refs[1]

    for(;;){
        // Test: remove later
        cout << "Leituras:\n\t L: " << readings[INDEX_LEFT] << "\n\t R: " << readings[INDEX_RIGHT] << endl;

        // Left motor control
        //   Error calculation
        err_left[control_idx.idx()] = refs[0] - readings[INDEX_LEFT];
        
        //   Control signal calculation
        u_left[control_idx.idx()] = 0.4806*err_left[control_idx.idx(-2)] + 0.4*u_left[control_idx.idx(-1)] + 0.6*u_left[control_idx.idx(-2)];

        //   Deadzone compensation
        left_voltage = anti_dz(u_left[control_idx.idx()], LEFT_DEADZONE_POS);

        //   Saturation
        left_voltage = saturate(left_voltage);

        // Right motor control
        //   Error calculation
        //   
        //  Given that the right encoder is not working, we will estimate the speed
        // err_right[control_idx.idx()] = refs[0] - readings[MOTOR_RIGHT];
        
        final_speed = K_DIR*u_right[control_idx.idx(-1)];

        // If it's the first measure
        if( abs(u_right[control_idx.idx(-1)] - u_right[control_idx.idx(-2)]) < 1e-4 ){
            actual_speed = final_speed + (actual_speed - final_speed)*exp(-CONTROLLER_PERIOD/TAU_DIR);
        }
        else {
            actual_speed = final_speed;
        };

        cout << "Simulacao direita: " << actual_speed << endl;

        err_right[control_idx.idx()] = refs[0] - actual_speed;

        //   Control signal calculation
        u_right[control_idx.idx()] = 0.4373*err_right[control_idx.idx(-2)] + 0.5075*u_right[control_idx.idx(-1)] + 0.4925*u_right[control_idx.idx(-2)];

        //   Deadzone compensation
        right_voltage = anti_dz(u_right[control_idx.idx()], RIGHT_DEADZONE_POS);

        //   Saturation
        right_voltage = saturate(right_voltage);

        // Test: remove later
        cout << "Erro:\n\t L: " << err_left[control_idx.idx()] << "\n\t R: " << err_right[control_idx.idx()] << endl;

        // Test: remove later
        cout << "Tensao:\n\t L: " << left_voltage << "\n\t R: " << right_voltage << endl;

        control_idx++;

        // Update PWM to be sent to arduino
        motor_set_voltage(MOTOR_LEFT, left_voltage, pwms);
        motor_set_voltage(MOTOR_RIGHT, right_voltage, pwms);

        rc_usleep(CONTROLLER_PERIOD*1000);
        
        if(rc_get_state() == EXITING)
            break;
    }

    return NULL;
}