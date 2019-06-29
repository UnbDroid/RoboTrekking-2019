#define ERRO_P  5
#define ERRO_N  -5

#define LIMIT   7

#include <cstdio>
#include <iostream>
#include <fstream>

extern "C" {
    #include <rc/encoder.h>
    #include <rc/encoder_eqep.h>
    #include <rc/time.h>
    #include <rc/adc.h>
    #include <rc/pthread.h>
}

int anti_dz(int tension){
    if(tension > 0)
        tension += ERRO_P;
    
    if(tension > LIMIT)
        tension = LIMIT;
}