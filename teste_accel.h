extern "C" {
    #include <stdio.h>
    #include <rc/mpu.h>
    #include <rc/time.h>
    #include <signal.h>
}


#define I2C_BUS 2

void accel2vel(rc_mpu_data_t *data, int * running);
void vel2pos(double * vel,int *running,uint64_t dt);