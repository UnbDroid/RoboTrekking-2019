#ifndef VISION_H_
#define VISION_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define MAX_DIST_FROM_CENTER
#define ANGLE_OF_MAX_DIST 35.0


typedef struct vision_thread_args{
    double accuracy;
    double direction;
    double angle;
} visonArgs;

void see_beyond(visonArgs* args);

#endif