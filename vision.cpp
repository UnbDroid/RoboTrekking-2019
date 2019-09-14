#include "vision.h"

using namespace cv;
using namespace std;

static VideoCapture cap(0);

void show(Mat image, String im)
{
  namedWindow(im, WINDOW_NORMAL);
  imshow(im, image);
  resizeWindow(im, 600, 400);
}

float findCone(Mat img_original, float* accuracy)
{
  Mat img_hsv, dilated, eroded, frame_threshold;  
  cvtColor(img_original, img_hsv, CV_BGR2HSV);
  inRange(img_hsv, Scalar(0, 38, 222), Scalar(15, 255, 255), frame_threshold);
  erode(frame_threshold, dilated, getStructuringElement(MORPH_RECT, Size(5, 5)));
  dilate(dilated, eroded, getStructuringElement(MORPH_RECT, Size(8, 8)));
  GaussianBlur(eroded, eroded, Size(3, 3), 0);
  int count = countNonZero(eroded);
  *accuracy = ( (float) count/(float)(eroded.cols * eroded.rows))*100.0;

  Moments m = moments(eroded,true);

  return 50-100*((m.m10/m.m00)/img_original.cols);
}

void see_beyond(visonArgs* args)
{
    Mat img_original;
    cap >> img_original;
    visonArgs* vision_arguments = (visonArgs*) args;
    vision_arguments->direction = findCone(img_original, &vision_arguments->accuracy);
    vision_arguments->angle = (-vision_arguments->direction * ANGLE_OF_MAX_DIST) / MAX_DIST_FROM_CENTER;
}