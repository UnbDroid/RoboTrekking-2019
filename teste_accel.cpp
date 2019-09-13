extern "C" {
    #include <stdio.h>
    #include <rc/mpu.h>
    #include <rc/time.h>
    #include <signal.h>
    #include <stdlib.h>
    #include <math.h>
}
#include <iostream>
#include "teste_accel.h"

#define I2C_BUS 2

using namespace std;

void accel2vel(rc_mpu_data_t *data, int * running) {
    u_int64_t t, dt;
    static double prev_accel[3], accel[3]= {0,0,0}, vel[3] = {0,0,0};
    static u_int64_t prev_t = rc_nanos_since_boot();

    //d_vel é a referência da velocidade para, a partir de uma tolerância, identificar o "drift"
    // e drift seria o acúmulo de drift na velocidade
    static int i = 1;
    static double d_vel[3] = {0,0,0}, drift[3] = {0,0,0};
    static u_int64_t d_t = prev_t, d_t_media = 0;
    static int drift_teste = 0, d_count = 0;
    prev_accel[0] = accel[0];
    prev_accel[1] = accel[1];
    prev_accel[2] = accel[2];

    accel[0] = (data->accel)[0];
    accel[1] = (data->accel)[1];
    accel[2] = (data->accel)[2];
    t = rc_nanos_since_boot();
    dt = t - prev_t;
   // prev_t = rc_nanos_since_boot();
    prev_t = t;

    vel[0] += (accel[0] + prev_accel[0])*dt/2e9;
    vel[1] += (accel[1] + prev_accel[1])*dt/2e9;
    vel[2] += (accel[2] + prev_accel[2])*dt/2e9;

    printf("__vel[0] = %.5lf __vel[1] = %.5lf __vel[2] = %.5lf\n", vel[0], vel[1], vel[2]);

    if(!drift_teste) {
            if ((abs(vel[0] - d_vel[0]) > 1) || (abs(vel[1] - d_vel[1]) > 1) || (abs(vel[2] - d_vel[2]) > 1)) {
            drift[0] += vel[0] - d_vel[0];
            drift[1] += vel[1] - d_vel[1];
            drift[2] += vel[2] - d_vel[2];

            d_vel[0] = vel[0];
            d_vel[1] = vel[1];
            d_vel[2] = vel[2];
            d_t_media += t - d_t;
            d_t = t;
            //cout<<i<<"\n";
            ++i;
        }

        if(i == 51) {
            drift[0] /= (i-1);
            drift[1] /= (i-1);
            drift[2] /= (i-1);
            d_t_media /= (i-1);
            printf("Drift médio = (%.5lf, %.5lf, %.5lf) e tempo médio = %.5lf\n",
            drift[0], drift[1], drift[2], d_t_media );

            printf("\nGravidade: %.5lf\n", sqrt(pow(drift[0],2)+pow(drift[1],2)+pow(drift[2],2)));
            vel[0] = 0;
            vel[1] = 0;
            vel[2] = 0;
            drift_teste = 1;
            //rc_usleep(3000000);
            d_t = rc_nanos_since_boot();
            *running = 0; //Para terminar execução do programa
        }
    } else {
        //d_vel agora é usada para compara a vel sem 'correçao' e a vel com 'correção'

        while(t - d_t > d_t_media) {
            d_t += d_t_media;
            d_count++;
            vel[0] -= drift[0];
            vel[1] -= drift[1];
            vel[2] -= drift[2];
            t = rc_nanos_since_boot();
            //printf("d_count = %d\n", d_count);
        }

       /* d_vel[0] = vel[0] - (d_count*drift[0]);
        d_vel[1] = vel[1] - (d_count*drift[1]);
        d_vel[2] = vel[2] - (d_count*drift[2]);*/
        //printf("d_vel[0] = %.5lf d_vel[1] = %.5lf d_vel[2] = %.5lf\n", d_vel[0], d_vel[1], d_vel[2]);
        //vel2pos(vel, running, dt);
        if (d_count==1000) {
            //*running = 0;
        printf("d_vel[0] = %.5lf d_vel[1] = %.5lf d_vel[2] = %.5lf\n", d_vel[0], d_vel[1], d_vel[2]);
        printf("__vel[0] = %.5lf __vel[1] = %.5lf __vel[2] = %.5lf\n", vel[0], vel[1], vel[2]);
        }
    }

}

void vel2pos(double * vel, int *running, uint64_t dt) {
    static double prev_vel[3] = {vel[0], vel[1], vel[2]};
    //u_int64_t dt;
    static double pst[3] = {0,0,0};
    //dt = t - prev_t;

    //prev_t = t;

    pst[0] += (vel[0] + prev_vel[0])*dt/2e9;
    pst[1] += (vel[1] + prev_vel[1])*dt/2e9;
    pst[2] += (vel[2] + prev_vel[2])*dt/2e9;
    
    prev_vel[0] = vel[0];
    prev_vel[1] = vel[1];
    prev_vel[2] = vel[2];

    printf("__pst[0] = %.5lf __pst[1] = %.5lf __pst[2] = %.5lf\n", pst[0], pst[1], pst[2]);

}