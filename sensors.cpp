#include "sensors.h"
#include "main.h"

using namespace std;

// Start MPU and return struct from where we can pool the data
static void start_mpu(rc_mpu_data_t* mpu_data){
    // MPU configuration struct
    rc_mpu_config_t mpu_config = rc_mpu_default_config();

    // Special configuration of config for dmp
	mpu_config.dmp_auto_calibrate_gyro = 0;
	mpu_config.dmp_sample_rate = 100;
    mpu_config.dmp_interrupt_sched_policy = SCHED_FIFO;
    mpu_config.dmp_interrupt_priority = 2;

    rc_mpu_initialize_dmp(mpu_data, mpu_config);
}

static void mpu_turnoff(void){
    rc_mpu_power_off();
}

void* filter_sensors(void *arg){
    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Castings
    volatile double* readings = (volatile double*)arg;

    // Speed related variables
    double  enc_l[2] = {0, 0},
            enc_r[2] = {0, 0},
            vel_l[2] = {0, 0}, 
            vel_r[2] = {0, 0};

    // Special kind of index
    index speed_idx = index(2, 0);

    // Time variables
    uint64_t time_spd, time_ref_spd = rc_nanos_since_boot();

    // MPU start
    rc_mpu_data_t mpu_data;
    start_mpu(&mpu_data);

    // TODO: Insert tap into gyro

    for(;;){

        // Get time of readings
        time_spd = rc_nanos_since_boot();

        // Raw reading from encoders
        enc_l[speed_idx.idx()] = rc_encoder_eqep_read(ENCODER_L);
        enc_r[speed_idx.idx()] = rc_encoder_eqep_read(ENCODER_R);

        // Conversion to meters
        enc_l[speed_idx.idx()] = abs( enc_l[speed_idx.idx()]*PI*DIAMETER/CPR );
        enc_r[speed_idx.idx()] = abs( enc_r[speed_idx.idx()]*PI*DIAMETER/CPR );

        // Discrete derivative low-pass filter to get speed, left into readings[2] and right into readings[3]
        //      meters/seconds (m/s)
        // Conversion to speed
        vel_l[speed_idx.idx()] = 30*enc_l[speed_idx.idx()] - 30*enc_l[speed_idx.idx(-1)] + exp(-30*(time_spd - time_ref_spd)/(double)1e9)*vel_l[speed_idx.idx(-1)];
        vel_r[speed_idx.idx()] = 30*enc_r[speed_idx.idx()] - 30*enc_r[speed_idx.idx(-1)] + exp(-30*(time_spd - time_ref_spd)/(double)1e9)*vel_r[speed_idx.idx(-1)];

        // Update arguments
        readings[2] = vel_l;
        readings[3] = vel_r;

        // Update reference of time
        time_ref_spd = time_spd;

        // Integrate accelerometer and encoders result and save into readings[0]
        //      meters (m)

        // Integrate gyro and store into readings[1], integration done by mpu dmp mode
        //      degrees (º)
        readings[1] = mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;
        cout << readings[1] << endl;
        
        // Sleep for some time
        rc_usleep(10);

        if(rc_get_state() == EXITING)
            break;    
    }    

    mpu_turnoff();

    return NULL;
}