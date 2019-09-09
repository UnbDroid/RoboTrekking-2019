#include <NewPing.h>

#define SONAR_NUM 4
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

// #define ALFA 0.8

#define US1_ECHO A4
#define US1_TRIG A5
#define US2_ECHO A4
#define US2_TRIG A5
#define US3_ECHO A4
#define US3_TRIG A5
#define US4_ECHO A4
#define US4_TRIG A5

NewPing sonar[SONAR_NUM] = {   
  NewPing(US1_TRIG, US1_ECHO, MAX_DISTANCE), 
  NewPing(US2_TRIG, US2_ECHO, MAX_DISTANCE), 
  NewPing(US3_TRIG, US3_ECHO, MAX_DISTANCE),
  NewPing(US4_TRIG, US4_ECHO, MAX_DISTANCE)
};
uint8_t us_byte;
#ifdef ALFA
float low_pass_filter(float last_val, float new_val)
{
  return ((ALFA)*last_val + (1.0-ALFA)*new_val);
}
#endif

void setup() {
  us_byte = 0x00;
}

//READ THE LINES BELOW !!!!!!!!!!!!

//the ALFA define activates and deactivates the low_pass_filter, if you need faster results and dont want to wait for the results to get stable you can delete the line defining it

//____________________________________________
//bits representation of ULTRASSONICS:

// | US3 | US2 | US1 | US0 |
// |b8 b7|b6 b5|b4 b3|b2 b1|

// 00 between 0 and 50 cms
// 01 between 50 and 100 cms
// 10 between 100 and 150 cms
// 11 between 150 and 200 cms or more 

#ifdef ALFA
uint8_t value_per_us(float value)
#else
uint8_t value_per_us(unsigned long value)
#endif
{
  if (value<50.0 && value >1.0)
    return 0x00;
  else if (value>50.0 && value <100.0)
    return 0x01;
  else if (value>100.0 && value<150.0)
    return 0x02;
  else
    return 0x03;
}

#ifdef ALFA
uint8_t mask(float* filtered_us)
#else
uint8_t mask(unsigned long* readings_us)
#endif
{
  uint8_t aux = 0x00;
  us_byte = 0x00;
  for (int8_t i = 3; i >= 0 ; i--)
  {
    #ifdef ALFA
      aux = value_per_us(filtered_us[i]);
    #else
      aux = value_per_us(readings_us[i]);
    #endif
    us_byte |= aux;
    if(i!=0)
      us_byte = us_byte << 0x02;
  }
  
}
//___________******************************************_________________________
// it is necessary to read the sensor at least CEIL(1/1-ALFA) for a stable result ... example (4 sensors will need 5 readings each)
//5x4 readings = 20 and each reading with the delay is about 50ms, reading all of them with filter and waiting for the stabilization will take about 1 second.
// if you disable the filter youll need only one rreading and then 4 x 50ms = 200ms

void read_us() //WARNING: TO GUARANTEE BEST RESULTS,  29ms should be the shortest delay between pings (keep the delay inside the loop)
{
  #ifdef ALFA
    static float filtered_us[4] = {0.0,0.0,0.0,0.0};
  #else
    unsigned long readings_us[4]={0,0,0,0};
  #endif
  
  #ifdef ALFA
  for (uint8_t j = 0; j < (uint8_t)(1/(1-ALFA))+1; j++)
  {
  #endif
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      #ifdef ALFA
        filtered_us[i] = low_pass_filter(filtered_us[i],sonar[i].ping_cm());
      #else
        readings_us[i] = sonar[i].ping_cm();
      #endif
      delay(28);
    }
  #ifdef ALFA
  }
  #endif
  #ifdef ALFA
    mask(filtered_us);
  #else
    mask(readings_us);
  #endif
}

void loop() { 
 read_us(); 
}
