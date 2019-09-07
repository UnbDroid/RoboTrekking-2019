#include "sensors.h"

using namespace std;

// Global to be accessed from dmp_callback
rc_mpu_data_t mpu_data;

volatile double drift;
volatile uint8_t drift_set = 0;

// Handle with the gyro error
static void handle_gyro_error(void){
    static double last_gyro = 0.0;
    double value;
    static double drift_m;

    static int32_t n_readings = 0;

    if(drift_set){
        drift += drift_m;
    }
    else{
        if(!n_readings){
            // Save first reading
            last_gyro = mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;
            ++n_readings;
        }
        else{
            // Save drift between consecutive readings
            value = mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;
            drift += value - last_gyro;
            last_gyro = value;
            ++n_readings;
            
            if(n_readings == COUNT_GYRO){
                drift_set = !drift_set;
                drift_m = drift / COUNT_GYRO;
            }
        }
    }
}

// Start MPU and return struct from where we can pool the data
static void start_mpu(){
    // MPU configuration struct
    rc_mpu_config_t mpu_config = rc_mpu_default_config();

    // Special configuration of config for dmp
	mpu_config.dmp_auto_calibrate_gyro = 0;
	mpu_config.dmp_sample_rate = 100;
    mpu_config.dmp_interrupt_sched_policy = SCHED_FIFO;
    mpu_config.dmp_interrupt_priority = 3;  // Do not modify

    rc_mpu_initialize_dmp(&mpu_data, mpu_config);

    rc_mpu_set_dmp_callback(handle_gyro_error);
}

static void mpu_turnoff(void){
    rc_mpu_power_off();
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

    mutex* navigation_mutex = args->arg_navigation_mutex;

    #if USING_ENCODER
    // Speed related variables
    double  enc_l[2] = {0, 0},
            enc_r[2] = {0, 0},
            vel_l[2] = {0, 0}, 
            vel_r[2] = {0, 0};

    double exp_result;

    // Special kind of index
    index enc_idx(2, 0);
    
    // Time variables
    uint64_t time_spd, time_ref_spd = rc_nanos_since_boot();

    uint64_t measure_period = (CONTROLLER_PERIOD/20)*1e9; // 20 measures per controller loop, in nanoseconds
    #endif

    // MPU start
    start_mpu();

    // Wait for gyro to calculate error
    while(!drift_set){continue;}

    unique_lock<mutex> navigation_lock(*navigation_mutex, defer_lock);

    // Sync
    unique_lock<mutex> control_lock(*control_mutex);
    control_cv->notify_one();
    control_lock.unlock();

    for(;;){

        #if USING_ENCODER
        // Get time of readings
        time_spd = rc_nanos_since_boot();

        // time to measure?
        if( (time_spd - time_ref_spd) > measure_period ){ 

            // Raw reading from encoders
            enc_l[enc_idx.idx()] = rc_encoder_eqep_read(ENCODER_L);
            enc_r[enc_idx.idx()] = rc_encoder_eqep_read(ENCODER_R);

            // Conversion to meters
            enc_l[enc_idx.idx()] = abs( enc_l[enc_idx.idx()]*PI*DIAMETER/CPR );
            enc_r[enc_idx.idx()] = abs( enc_r[enc_idx.idx()]*PI*DIAMETER/CPR );

            // Discrete derivative low-pass filter to get speed, left into readings[2] and right into readings[3]
            //      meters/seconds (m/s)
            // Conversion to speed

            // The derivative filter here is 10s/(s + 10)

            exp_result = exp(-10.0*((time_spd - time_ref_spd)/1e9));

            vel_l[enc_idx.idx()] = 10*(enc_l[enc_idx.idx()] - enc_l[enc_idx.idx(-1)]) + exp_result*vel_l[enc_idx.idx(-1)];
            vel_r[enc_idx.idx()] = 10*(enc_r[enc_idx.idx()] - enc_r[enc_idx.idx(-1)]) + exp_result*vel_r[enc_idx.idx(-1)];

            // Lock for safety updating
            control_lock.lock();

            // Update arguments
            readings[2] = vel_l[enc_idx.idx()];
            readings[3] = vel_r[enc_idx.idx()];
        
            // Unlock
            control_lock.unlock();

            // Increment index
            enc_idx++;

            // Update reference of time
            time_ref_spd = time_spd;
        }
        #endif

        // Integrate accelerometer and encoders result and save into readings[0]
        //      meters (m)

        // Integrate gyro and store into readings[1], integration done by mpu dmp mode
        //      degrees (º)
        navigation_lock.lock();

        //readings[1] = mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;
        readings[1] = mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG - drift;

        navigation_lock.unlock();

        // Sleep for some time (0.005 seconds)
        rc_usleep(5000);

        if(rc_get_state() == EXITING)
            break;    
    }    

    mpu_turnoff();

    return NULL;
}