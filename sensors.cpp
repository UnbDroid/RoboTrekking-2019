#include "sensors.h"

using namespace std;

#define COUNT_GYRO 10;

volatile double gyro_error = 0;
volatile double t_error_gyro = 0;

// Time variables
volatile uint64_t time_spd = 0, time_ref_spd = 0;

// Start MPU and return struct from where we can pool the data
static void start_mpu(rc_mpu_data_t* mpu_data){
    // MPU configuration struct
    rc_mpu_config_t mpu_config = rc_mpu_default_config();

    volatile vector<double> gyro_readings;
    volatile double last_gyro = 0;

    // Calibrate gyro
    if(rc_mpu_calibrate_gyro_routine(config)<0){
        printf("Failed to complete gyro calibration\n");
        return -1;
    }

    // Special configuration of config for dmp
	mpu_config.dmp_auto_calibrate_gyro = 0;
	mpu_config.dmp_sample_rate = 100;
    mpu_config.dmp_interrupt_sched_policy = SCHED_FIFO;
    mpu_config.dmp_interrupt_priority = 1;

    for (size_t i = ; i <= COUNT_GYRO; i++)
    {
        gyro_readings.push_back(data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;);
    }

    for (double value:gyro_readings)
    {
        if(last_gyro == 0)
            last_gyro = value;
        
        else{
            gyro_error += last_gyro - value;
            last_gyro = value;
        }
    }
    
    gyro_error /= COUNT_GYRO;

    rc_mpu_initialize_dmp(mpu_data, mpu_config);
}

static void mpu_turnoff(void){
    rc_mpu_power_off();
}

static void handle_gyro_error(void){
    t_error_gyro += gyro_error;
}

void* filter_sensors(void *arg){
    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Castings
    sensorsArgs* args = (sensorsArgs*)arg;
    volatile double* readings = (double*)args->arg_readings;

    // Sync variables
    mutex* control_mutex = args->arg_control_mutex;
    condition_variable* control_cv = args->arg_control_cv;

    // Speed related variables
    double  enc_l[2] = {0, 0},
            enc_r[2] = {0, 0},
            vel_l[2] = {0, 0}, 
            vel_r[2] = {0, 0};

    double exp_result;

    int number_of_gyro_readings = 0;

    // Special kind of index
    index speed_idx(2, 0);
    index enc_idx(2, 0);

    // MPU start
    rc_mpu_data_t mpu_data;
    start_mpu(&mpu_data);

    // Sync
    unique_lock<mutex> control_lock(*control_mutex);
    control_cv->notify_one();
    control_lock.unlock();

    time_ref_spd = rc_nanos_since_boot();

    for(;;){

        // Get time of readings
        time_spd = rc_nanos_since_boot();

        // Correct gyro error
        if(number_of_gyro_readings > COUNT_GYRO){
            rc_mpu_set_dmp_callback(&handle_gyro_error);
            number_of_gyro_readings = 0;    
        }

        // 0.002 seconds
        if( (time_spd - time_ref_spd) > 2000 ){ 

            // Raw reading from encoders
            enc_l[enc_idx.idx()] = rc_encoder_eqep_read(ENCODER_L);
            enc_r[enc_idx.idx()] = rc_encoder_eqep_read(ENCODER_R);

            // Conversion to meters
            enc_l[enc_idx.idx()] = abs( enc_l[enc_idx.idx()]*PI*DIAMETER/CPR );
            enc_r[enc_idx.idx()] = abs( enc_r[enc_idx.idx()]*PI*DIAMETER/CPR );

            // Ignore fast changes
            if( !((enc_l[enc_idx.idx()] - enc_l[enc_idx.idx(-1)]) > 1 ) && !((enc_r[enc_idx.idx()] - enc_r[enc_idx.idx(-1)]) > 1 ) ){        

                // Discrete derivative low-pass filter to get speed, left into readings[2] and right into readings[3]
                //      meters/seconds (m/s)
                // Conversion to speed

                exp_result = exp(-30.0*((time_spd - time_ref_spd)/1e9));

                vel_l[speed_idx.idx()] = 30*(enc_l[enc_idx.idx()] - enc_l[enc_idx.idx(-1)]) + exp_result*vel_l[speed_idx.idx(-1)];
                vel_r[speed_idx.idx()] = 30*enc_r[enc_idx.idx()] - 30*enc_r[enc_idx.idx(-1)] + exp_result*vel_r[speed_idx.idx(-1)];

                // Lock for safety updating
                control_lock.lock();

                // Update arguments
                readings[2] = vel_l[enc_idx.idx()];
                readings[3] = vel_r[enc_idx.idx()]; 
            
                // Unlock
                control_lock.unlock();

                // Increment index
                speed_idx++;
            }

            enc_idx++;

            // Update reference of time
            time_ref_spd = time_spd;
        }

        // Integrate accelerometer and encoders result and save into readings[0]
        //      meters (m)

        // Integrate gyro and store into readings[1], integration done by mpu dmp mode
        //      degrees (º)
        readings[1] = mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG - t_error_gyro;
        ++number_of_gyro_readings;
        
        // Sleep for some time
        rc_usleep(50);

        if(rc_get_state() == EXITING)
            break;    
    }    

    mpu_turnoff();

    return NULL;
}