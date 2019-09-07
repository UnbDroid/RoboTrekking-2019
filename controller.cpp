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

// Maximum was choosen for this application simply as 7, nor voltage nor speed reference can surpass 7
static double saturate(double signal, double min, double max=MAXIMUM_VOLTAGE){
    if(signal > max)
        signal = max;
    else if(signal < min)
        signal = min;

    return signal;
}

// Dumb PID controller for gyro action
static double gyro_action(double gyro_ref, double gyro_limit){
    // Variables for gyro error
    // Assuming the average error to be 45 degrees, the proportional gain will be on the range (0 - 0.03)
    // Given the same considerations, the incremental gain will be magically chosen as 0.0025
    // The derivative gain will be based on a straight line and on a 45 degrees turn, for a straight line, expect a variation of 0.1, and during a turn
    //      a variation of 10 degrees per measure, a gain of 0.4 was choosen  
    static double gyro_p = 0.08, gyro_i = 0.0025, gyro_d = 0.05, sum_gyro_error = 0, last_gyro = 0; 
    static double action;

    action = gyro_p*gyro_ref + sum_gyro_error + gyro_d*(gyro_ref - last_gyro);
    sum_gyro_error += gyro_i*gyro_ref;

    last_gyro = gyro_ref;

    // Limit gyro action
    if(action > gyro_limit)
        return gyro_limit;
    else if(action < -gyro_limit)
        return -gyro_limit;
    return action;
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
    mutex* refs_mutex = control_arguments->arg_refs_mutex;

    // Variables to safely access other thread values
    double speed_ref[2], gyro_ref = 0;
    
    // Gyro influence over references
    double gyro_PID;

    #if USING_ENCODER
    double speed_readings[2] = {0.0, 0.0};

    // Weight of importance to give to the readings from the encoder, between 0 and 1, 0 index is LEFT, 1 is RIGHT
    double readings_weight[2] = {1.0, 0.0}; 
    #endif

    // Circular arrays for control calculations
    double err_left[3] = {0, 0, 0},
           err_right[3] = {0, 0, 0},
           u_left[3] = {0, 0, 0},
           u_right[3] = {0, 0, 0};

    // Variables used because the encoder readings cannot be trusted 100%
    double final_speed[2], simu_speed[2] = {0.0, 0.0};

    // Special kind of index
    index control_idx(3);

    // Sync with sensors thread
    unique_lock<mutex> sensors_lock(*sensors_mutex);
    sensors_cv->wait(sensors_lock);
    sensors_lock.unlock();

    unique_lock<mutex> refs_lock(*refs_mutex, defer_lock);

    for(;;){

        // Simulate speed readings, in case encoders cannot be trusted (DO NOT TRUST THEM, THEY LIE!)
        
        /*-----------*/
        // Left side //
        /*-----------*/
        
        final_speed[INDEX_LEFT] = K_LEFT*u_left[control_idx.idx(-1)];

        // If it's the first measure from last change of output
        if( abs(u_left[control_idx.idx(-1)] - u_left[control_idx.idx(-2)]) < 1e-2 ){
            simu_speed[INDEX_LEFT] = final_speed[INDEX_LEFT] + (simu_speed[INDEX_LEFT] - final_speed[INDEX_LEFT])*exp(-CONTROLLER_PERIOD/TAU_LEFT);
        }
        else {
            simu_speed[INDEX_LEFT] = final_speed[INDEX_LEFT];
        };

        /*------------*/
        // Right side //
        /*------------*/

        final_speed[INDEX_RIGHT] = K_DIR*u_right[control_idx.idx(-1)];
        
        // If it's the first measure
        if( abs(u_right[control_idx.idx(-1)] - u_right[control_idx.idx(-2)]) < 1e-2 ){
            simu_speed[INDEX_RIGHT] = final_speed[INDEX_RIGHT] + (simu_speed[INDEX_RIGHT] - final_speed[INDEX_RIGHT])*exp(-CONTROLLER_PERIOD/TAU_DIR);
        }
        else {
            simu_speed[INDEX_RIGHT] = final_speed[INDEX_RIGHT];
        };

        // Get values from other threads safely
        sensors_lock.lock();
        refs_lock.lock();
        
        #if USING_ENCODER
        speed_readings[INDEX_LEFT] = readings[INDEX_LEFT];
        speed_readings[INDEX_RIGHT] = readings[INDEX_RIGHT];
        #endif

        speed_ref[INDEX_LEFT] = refs[0];
        gyro_ref = refs[1];

        refs_lock.unlock();
        sensors_lock.unlock();

        // Update speed reference based on gyro error
        //
        // If error is positive, measured angle is lower than desired, robot must turn anti-clockwise
        //      To turn anti-clockwise, left speed must decrease while right speed must increase 
        // If error is negative, measured angle is higher than desired, robot must turn clockwise
        //      To turn clockwise, left speed must increase while right speed must decrease
        //
        // The maximum influence of the gyro will be 80% of the speed reference
        gyro_PID = gyro_action(gyro_ref, speed_ref[INDEX_LEFT]*0.9);
        speed_ref[INDEX_RIGHT] = speed_ref[INDEX_LEFT] + gyro_PID;
        speed_ref[INDEX_LEFT] -= gyro_PID;

        // Avoid getting less than 0 values
        speed_ref[INDEX_LEFT] = saturate(speed_ref[INDEX_LEFT], 0);
        speed_ref[INDEX_RIGHT] = saturate(speed_ref[INDEX_RIGHT], 0);


        // If encoders readings can be trusted up to some level
        #if USING_ENCODER
            // Update error with base on readings
            err_left[control_idx.idx()] = speed_ref[INDEX_LEFT] - (speed_readings[INDEX_LEFT]*readings_weight[INDEX_LEFT] + (1-readings_weight[INDEX_LEFT])*simu_speed[INDEX_LEFT]);
            err_right[control_idx.idx()] = speed_ref[INDEX_RIGHT] - (speed_readings[INDEX_RIGHT]*readings_weight[INDEX_RIGHT] + (1-readings_weight[INDEX_RIGHT])*simu_speed[INDEX_RIGHT]);
        #else
            // Encoders readings cannot be trusted AT ALL            
            err_left[control_idx.idx()] = speed_ref[INDEX_LEFT] - simu_speed[INDEX_LEFT];
            err_right[control_idx.idx()] = speed_ref[INDEX_RIGHT] - simu_speed[INDEX_RIGHT];
        #endif

        /*--------------------*/
        // Left motor control //
        /*--------------------*/

        //   Control signal calculation
        u_left[control_idx.idx()] = 0.4806*err_left[control_idx.idx(-2)] + 0.4*u_left[control_idx.idx(-1)] + 0.6*u_left[control_idx.idx(-2)];

        //   Deadzone compensation
        left_voltage = anti_dz(u_left[control_idx.idx()], LEFT_DEADZONE_POS);

        //   Saturation
        left_voltage = saturate(left_voltage, MINIMUM_VOLTAGE, MAXIMUM_VOLTAGE);

        /*---------------------*/
        // Right motor control //
        /*---------------------*/

        //   Control signal calculation
        u_right[control_idx.idx()] = 0.4373*err_right[control_idx.idx(-2)] + 0.5075*u_right[control_idx.idx(-1)] + 0.4925*u_right[control_idx.idx(-2)];

        //   Deadzone compensation
        right_voltage = anti_dz(u_right[control_idx.idx()], RIGHT_DEADZONE_POS);

        //   Saturation
        right_voltage = saturate(right_voltage, MINIMUM_VOLTAGE, MAXIMUM_VOLTAGE);

        cout << gyro_ref << ' ' << speed_ref[INDEX_LEFT] << ' ' << speed_ref[INDEX_RIGHT] << endl;
        cout << "\t Simu: " << simu_speed[INDEX_LEFT] << ' ' << simu_speed[INDEX_RIGHT] << endl;
        cout << "\t \t U: " << u_left[control_idx.idx()] << ' ' << u_right[control_idx.idx()] << endl;
        cout << "\t \t \t Voltage: " << left_voltage << ' ' << right_voltage << endl;

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