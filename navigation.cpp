#include "navigation.h"

volatile State state = START;

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
volatile double targets_position[6] = {40, 20, 30, 2, 6, 18}

// Return the angle between two points
double angle_from_positions(double x1, double y1, double x2, double y2){
    double d_x = x1 - x2;
    double d_y = y1 - y2;

    return atan2(d_y, d_x) * 180 / PI;
}

// Update robot position acording to the reference from general_readings in main.cpp
void update_robot_position(double distance, double angle){
    robot_position[0] = distance * sin(angle * PI/180);
    robot_position[1] = distance * cos(angle * PI/180);
}

// Update the targets position acording to distance and angle received from vision and using
// the robot position
void update_targets(double distance, double angle, int target){

}

void* navigation_control(void* args){
    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

    // Number of targets reached
    int targets = 0;

    // Flags
    bool saw_traffic_cone = false;
    bool reached_target = false;
    bool targets_updated = false;

    // Casting
    navigationArgs* navigation_arguments = (navigationArgs*)args;
    volatile double* ref[2] = (volatile double*) navigation_arguments->arg_refs;
    volatile double* readings[4] = (volatile double*) navigation_arguments->arg_g_readings;

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
            ref[0] = MAX_SPEED;

            if(saw_traffic_cone){
                if(!targets_updated){
                    // update_targets(distance_to_cone(), angle_to_cone());
                    targets_updated = true;
                }
                // ref[1] += angle_to_cone();
            }
            break;
            
        case GO_TO_SECOND:
            break;
            
        case GO_TO_LAST:
            if(something_neer()){
                state = DODGE;
            }

            break;
            
        case GO_ROUND:
            saw_traffic_cone = false;
            reached_target = false;
            targets_updated = false;

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

        if(reached_target){
            // turn_on_the_lights();
            targets++;
            if(targets < 3)
                state = GO_ROUND;
            else
                state = END;
        }

        update_robot_position();

        rc_usleep(PERIOD*1000);
    }
}