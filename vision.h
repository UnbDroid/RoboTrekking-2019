#ifndef VISION_H_
#define VISION_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


typedef struct vision_thread_args{
    float* vis_accuracy;
    float* vis_direction;
} visonArgs;

void* vision_thread(void* args);

#endif