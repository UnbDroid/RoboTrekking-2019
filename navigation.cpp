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
double targets_position[6] = {40, 20, 30, 2, 6, 18}

// Return the angle between two points
double angle_from_positions(double x1, double y1, double x2, double y2){
    double d_x = x1 - x2;
    double d_y = y1 - y2;

    return atan2(d_y, d_x) * 180 / PI;
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

// Update the robot position acording to angle received from vision and using
// the target position
void update_targets(double angle){

}

void* navigation_control(void* args){
    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Number of targets reached
    int targets = 0;
    // Times that the robot changed the angle to go around the cone
    int times_change_ang = 0;

    // Flags
    bool saw_traffic_cone = false;
    bool reached_target = false;
    bool targets_updated = false;
    bool started_go_around = false

    // Casting
    navigationArgs* navigation_arguments = (navigationArgs*)args;
    volatile double* ref[2] = (volatile double*) navigation_arguments->arg_refs;
    volatile double* readings[4] = (volatile double*) navigation_arguments->arg_g_readings;

    visonArgs vision_arguments;

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
            //TODO: when neerby the cone, set a flag that start the vision thread
            

            if(distance_betwen_two_points(  robot_position[0],
                                            robot_position[1],
                                            targets_position[0],
                                            targets_position[1]) <= DISTANCE_TO_USE_VISION){
                ref[0] = APROX_SPEED;
                vision_funct(&vision_arguments);
                //if(vision_arguments->accuracy)
                //TODO: correct robot position acording to angle
                if(distace_US() < DISTANCE_TO_START_GO_AROUND){
                    ref[0] = CIRCLE_SPEED;
                    state = GO_AROUND;
                    targets++;             
                }
            }
            break;
            
        case GO_TO_SECOND:
            //TODO: when neerby the cone, set a flag that start the vision thread
            

            if(distance_betwen_two_points(  robot_position[0],
                                            robot_position[1],
                                            targets_position[2],
                                            targets_position[3]) <= DISTANCE_TO_START_GO_AROUND){
                ref[0] = APROX_SPEED;
                vision_funct(&vision_arguments);
                //if(vision_arguments->accuracy)
                //TODO: correct robot position acording to angle
                if(distace_US() < DISTANCE_TO_START_GO_AROUND){
                    ref[0] = CIRCLE_SPEED;
                    state = GO_AROUND;
                    targets++;             
                }
            }
            break;
            
        case GO_TO_LAST:
            if(something_neer()){
                state = DODGE;
            }
            //TODO: when neerby the cone, set a flag that start the vision thread
            

            if(distance_betwen_two_points(  robot_position[0],
                                            robot_position[1],
                                            targets_position[4],
                                            targets_position[5]) <= DISTANCE_TO_START_GO_AROUND){
                ref[0] = APROX_SPEED;
                vision_funct(&vision_arguments);
                //if(vision_arguments->accuracy)
                //TODO: correct robot position acording to angle
                if(distace_US() < DISTANCE_TO_START_GO_AROUND){
                    ref[0] = CIRCLE_SPEED;
                    state = END;         
                }
            }
            break;
            
        case GO_AROUND:
            saw_traffic_cone = false;
            reached_target = false;
            targets_updated = false;
            //TODO: set the vision flag to make the vision thread stop

            ref[0] = CIRCLE_SPEED;
            if(!started_go_around){
                ref[1] -= OFFSET_ANGLE_TO_START_CIRCLE;
                started_go_around = true;
            }
            
            if(times_change_angl < 2){
                if(readings[0] >= DISTANCE_TO_GO_AROUND){
                    times_change_angl++;
                    ref[1] += ANGLE_TO_GO_AROUND;
                }
            }
            else{
                if(targets == 1){
                    state = GO_TO_SECOND;
                    // Where the robot is: (40,20); where it needs to go: (30,2)
                    ref[1] = angle_from_positions(  robot_position[0],
                                                    robot_position[1],
                                                    targets_position[2],
                                                    targets_position[3]);
                }
                else{
                    state = GO_TO_LAST;
                    // Where the robot is: (30,2); where it needs to go: (6,18)
                    ref[1] = angle_from_positions(  robot_position[0],
                                                    robot_position[1],
                                                    targets_position[4],
                                                    targets_position[5]);
                }
            }
            break;

        case DODGE:
            break;
            
        case END:
            ref[0] = STOP;
            break;
        
        default:
            state = END;
            break;
        }

        update_robot_position(readings[0], readings[1]);
        readings[0] = 0;

        rc_usleep(NAVIGATION_PERIOD*1000);
    }
}