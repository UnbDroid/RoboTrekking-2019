#include "navigation.h"
#include "vision.h"

State state = START;

// Save the actual robot position on the matrix
//      0 -> x position
//      1 -> y position
volatile double robot_position[2] = {3, 3};

// Save the targets position on the matrix
//      0 -> x position of first target
//      1 -> y position of first target
//      2 -> x position of second target
//      3 -> y position of second target
//      4 -> x position of third and last target
//      5 -> y position of third and last target
double targets_position[6] = {40, 20, 30, 2, 6, 18};

// Boolean vector that contain if US see something in a predeterminated range or not
//      0 -> left US
//      1 -> left center US
//      2 -> right center US
//      3 -> right US
volatile bool us_readings[4] = {false, false, false, false};

// Range for angle
double angle_range = ANGLE_TO_ADD / 2;

// Return the angle between two points
double angle_from_positions(double x1, double y1, double x2, double y2){
    double d_x = x1 - x2;
    double d_y = y1 - y2;
    double angle = atan2(d_y, d_x) * 180 / PI;
    if(angle > 90)
      angle -= 360;

    return angle;
}

double distance_betwen_two_points(double x1, double y1, double x2, double y2){
    double d_x = x1 - x2;
    double d_y = y1 - y2;

    return sqrt(pow(d_x,2) + pow(d_y,2));
}

// Update robot position acording to the reference from general_readings in main.cpp
void update_robot_position(double distance, double angle){
    robot_position[0] += distance * sin(angle * PI/180);
    robot_position[1] += distance * cos(angle * PI/180);
}

// Verify if US finds cone
bool US_find_cone(){
    return(us_readings[0] || us_readings[1] || us_readings[2] || us_readings[3]);
}

void* navigation_control(void* args){
    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Number of targets reached
    int targets = 0;
    // Times that the robot changed the angle to go around the cone
    int times_change_ang = 0;

    // Flag
    bool started_go_around = false;
    bool seen_cone = false;

    // Casting
    navigationArgs* navigation_arguments = (navigationArgs*)args;
    volatile double* ref[2] = (volatile double*) navigation_arguments->arg_refs;
    volatile double* readings[4] = (volatile double*) navigation_arguments->arg_g_readings;
    volatile double* distance_travelled = (volatile double*) navigation_arguments->arg_distance;

    visonArgs vision_arguments;
    VideoCapture cap(0);
    if(!cap.isOpened()){
        return NULL;
    }

    for(;;){
        switch (state)
        {
        case START:
            ref[0] = MAX_SPEED;
            // Where the robot is: (3,3); where it needs to go: (40,20)
            ref[1] = angle_from_positions(  robot_position[0],
                                            robot_position[1],
                                            targets_position[0],
                                            targets_position[1]);

            state = GO_TO_FIRST;
            break;
            
        case GO_TO_FIRST:
            if(distance_betwen_two_points(  3, 3,
                                            targets_position[0], targets_position[1])
            - distance_travelled <= DISTANCE_TO_USE_VISION){
                if(distance_betwen_two_points(  3, 3,
                                                targets_position[0], targets_position[1])
                - distance_travelled < DISTANCE_TO_START_GO_AROUND){
                    if(US_find_cone()){
                        ref[0] = CIRCLE_SPEED;
                        state = GO_AROUND;
                        targets++;
                        break;            
                    } 
                }
                ref[0] = APROX_SPEED;
                see_beyond(&vision_arguments);
                if(vision_arguments->accuracy > IDEAL_ACCURACY && !seen_cone){
                    // Add a offset angle to correct route to cone
                    ref[1] += vision_arguments->angle;
                    seen_cone = true;
                }
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case GO_TO_SECOND:
            if(distance_betwen_two_points(  targets_position[0], targets_position[1],
                                            targets_position[2], targets_position[3])
            - distance_travelled <= DISTANCE_TO_USE_VISION){
                if(distance_betwen_two_points(  targets_position[0], targets_position[1],
                                                targets_position[2], targets_position[3])
                - distance_travelled < DISTANCE_TO_START_GO_AROUND){
                    if(US_find_cone()){
                        ref[0] = CIRCLE_SPEED;
                        state = GO_AROUND;
                        targets++;
                        break;            
                    } 
                }
                ref[0] = APROX_SPEED;
                see_beyond(&vision_arguments);
                if(vision_arguments->accuracy > IDEAL_ACCURACY && !seen_cone){
                    // Add a offset angle to correct route to cone
                    ref[1] += vision_arguments->angle;
                    seen_cone = true;
                }
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case GO_TO_LAST:
            if(distance_betwen_two_points(  targets_position[2], targets_position[3],
                                            targets_position[4], targets_position[5])
            - distance_travelled <= DISTANCE_TO_USE_VISION){
                if(distance_betwen_two_points(  targets_position[2], targets_position[3],
                                                targets_position[4], targets_position[5])
                - distance_travelled < DISTANCE_TO_START_GO_AROUND){
                    if(US_find_cone()){
                        state = END;
                        break;            
                    } 
                }         
                ref[0] = APROX_SPEED;
                see_beyond(&vision_arguments);
                if(vision_arguments->accuracy > IDEAL_ACCURACY && !seen_cone){
                    // Add a offset angle to correct route to cone
                    ref[1] += vision_arguments->angle;
                    seen_cone = true;
                }
            }
            if(us_readings[0] || us_readings[1] || us_readings[2] || us_readings[3]){
                state = DODGE;
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case GO_AROUND:
            ref[0] = CIRCLE_SPEED;
            seen_cone = false;
            
            if(ref[1] < angle_from_positions(targets_position[0], targets_position[1], targets_position[2], targets_position[3]) + angle_range
               && ref[1] > angle_from_positions(targets_position[0], targets_position[1], targets_position[2], targets_position[3]) - angle_range){
                if(targets == 1){
                    state = GO_TO_SECOND;
                    // Where the robot is: (40,20); where it needs to go: (30,2)
                    ref[1] = angle_from_positions(  targets_position[0],
                                                    targets_position[1],
                                                    targets_position[2],
                                                    targets_position[3]);
                }
                else{
                    state = GO_TO_LAST;
                    // Where the robot is: (30,2); where it needs to go: (6,18)
                    ref[1] = angle_from_positions(  targets_position[2],
                                                    targets_position[3],
                                                    targets_position[4],
                                                    targets_position[5]);
                }
            }
            else{
                ref[1] -= ANGLE_TO_ADD;
            }
            break;

        case DODGE:
            if(us_readings[0] || us_readings[1]){
                if(us_readings[1])
                    ref[1] -= BIG_ANGLE_TO_DODGE;
                else
                    ref[1] -= SMALL_ANGLE_TO_DODGE;
            }
            else if(us_readings[2] || us_readings[3]){
                if(us_readings[2])
                    ref[1] += BIG_ANGLE_TO_DODGE;
                else
                    ref[1] += SMALL_ANGLE_TO_DODGE;
            }
            else{
                state = GO_TO_LAST;
                // Where the robot is: (30,2); where it needs to go: (6,18)
                ref[1] = angle_from_positions(  robot_position[0],
                                                robot_position[1],
                                                targets_position[4],
                                                targets_position[5]);
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case END:
            ref[0] = STOP;
            break;
        
        default:
            state = END;
            break;
        }

        
        readings[0] = 0;

        rc_usleep(NAVIGATION_PERIOD*1000);
    }
}