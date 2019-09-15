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
volatile bool us_readings[4] = {true, false, false, false};

// Range for angle
double angle_range = ANGLE_TO_ADD;

// Return the angle between two points
double angle_from_positions(double x1, double y1, double x2, double y2){
    double d_x = x2 - x1;
    double d_y = y2 - y1;
    double angle = atan2(d_y, d_x) * 180 / PI;
    if(angle > 90)
      angle -= 360;
    return angle - 24.67;
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

    // Flag
    bool seen_cone = false;

    // Casting
    navigationArgs* navigation_arguments = (navigationArgs*)args;
    double* ref = navigation_arguments->arg_refs;
    double* readings = navigation_arguments->arg_g_readings;
    bool* us_flag = (bool*) navigation_arguments->arg_us;

    // Sync
    mutex* refs_mutex = navigation_arguments->arg_refs_mutex;
    mutex* sensors_mutex = navigation_arguments->arg_sensors_mutex;

    unique_lock<mutex> refs_lock(*refs_mutex, defer_lock);
    unique_lock<mutex> sensors_lock(*sensors_mutex, defer_lock);

    visonArgs vision_arguments;

    double distance_left;
    double desiered_angle;
    double angle_error;

    for(;;){
        switch (state)
        {
        case START:
            angle_error = angle_from_positions(  robot_position[0],
                                            robot_position[1],
                                            targets_position[0],
                                            targets_position[1]);
            if(readings[1] > 0.001 || readings[1] < -0.001 ){
                sensors_lock.lock();
                refs_lock.lock();
                
                ref[0] = MAX_SPEED;
                // Where the robot is: (3,3); where it needs to go: (40,20)
                ref[1] = angle_error - readings[1];
                state = GO_TO_FIRST;

                cout << "Started" << readings[1] << endl;
                refs_lock.unlock();
                sensors_lock.unlock();
            }
            break;
            
        case GO_TO_FIRST:
            sensors_lock.lock();
            refs_lock.lock();
            ref[1] = angle_error - readings[1];
            refs_lock.unlock();
            sensors_lock.unlock();
            distance_left = distance_betwen_two_points(3, 3, targets_position[0], targets_position[1]) - readings[0];
            if(distance_left <= DISTANCE_TO_USE_VISION){
                cout << "Vision On" << endl;
                if(distance_left < DISTANCE_TO_START_GO_AROUND){
                    cout << "US On" << endl;
                    if(!*us_flag){
                        cout << "IN HERE" << endl;
                        *us_flag = true;
                    }
                    if(US_find_cone() || distance_left < DISTANCE_ARRIVED){
                        refs_lock.lock();
                        ref[0] = CIRCLE_SPEED;
                        refs_lock.unlock();

                        state = GO_AROUND;
                        cout << "Reached 1" << endl;
                        // state = END;
                        targets++;
                        break;            
                    } 
                }
                refs_lock.lock();
                ref[0] = APROX_SPEED;
                refs_lock.unlock();

                see_beyond(&vision_arguments);
                if( (vision_arguments.accuracy > IDEAL_ACCURACY) && !seen_cone){
                    cout << "Above ideal accuracy" << endl << "\t adding " << (double)vision_arguments.angle;
                    // Add a offset angle to correct route to cone

                    refs_lock.lock();
                    ref[1] += (double)vision_arguments.angle;
                    refs_lock.unlock();
                    
                    seen_cone = true;
                }
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case GO_TO_SECOND:
            sensors_lock.lock();
            refs_lock.lock();
            ref[1] = angle_from_positions(  targets_position[0],
                                            targets_position[1],
                                            targets_position[2],
                                            targets_position[3]) - readings[1];
            refs_lock.unlock();
            sensors_lock.unlock();
            if(distance_betwen_two_points(targets_position[0], targets_position[1], targets_position[2], targets_position[3]) - readings[0] <= DISTANCE_TO_USE_VISION){
                if(distance_betwen_two_points(targets_position[0], targets_position[1], targets_position[2], targets_position[3]) - readings[0] < DISTANCE_TO_START_GO_AROUND){
                    if(!us_flag)
                        *us_flag = true;
                    if(US_find_cone()){
                        refs_lock.lock();
                        ref[0] = CIRCLE_SPEED;
                        refs_lock.unlock();

                        state = GO_AROUND;
                        cout << "Reached 2" << endl;
                        targets++;
                        break;            
                    } 
                }
                refs_lock.lock();
                ref[0] = APROX_SPEED;
                refs_lock.unlock();

                see_beyond(&vision_arguments);
                if(vision_arguments.accuracy > IDEAL_ACCURACY && !seen_cone){
                    // Add a offset angle to correct route to cone
                    refs_lock.lock();
                    ref[1] += (double)vision_arguments.angle;
                    refs_lock.unlock();
                    seen_cone = true;
                }
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case GO_TO_LAST:
            sensors_lock.lock();
            refs_lock.lock();
            ref[1] = angle_from_positions(  targets_position[2],
                                            targets_position[3],
                                            targets_position[4],
                                            targets_position[5]) - readings[1];
            refs_lock.unlock();
            sensors_lock.unlock();
            if(!us_flag)
                *us_flag = true;
            if(distance_betwen_two_points(targets_position[2], targets_position[3],targets_position[4], targets_position[5]) - readings[0] <= DISTANCE_TO_USE_VISION){
                if(distance_betwen_two_points(targets_position[2], targets_position[3], targets_position[4], targets_position[5]) - readings[0] < DISTANCE_TO_START_GO_AROUND){
                    if(US_find_cone()){
                        state = END;
                        break;            
                    } 
                }
                refs_lock.lock();
                ref[0] = APROX_SPEED;
                refs_lock.unlock();

                see_beyond(&vision_arguments);
                if(vision_arguments.accuracy > IDEAL_ACCURACY && !seen_cone){
                    // Add a offset angle to correct route to cone
                    refs_lock.lock();
                    ref[1] += (double)vision_arguments.angle;
                    refs_lock.unlock();
                    seen_cone = true;
                }
            }
            if(us_readings[0] || us_readings[1] || us_readings[2] || us_readings[3]){
                state = DODGE;
            }
            update_robot_position(readings[0], readings[1]);
            break;
            
        case GO_AROUND:
            refs_lock.lock();
            ref[0] = CIRCLE_SPEED;
            refs_lock.unlock();
            seen_cone = false;
            *us_flag = false;
            desiered_angle = angle_from_positions(targets_position[0], targets_position[1], targets_position[2], targets_position[3]);
            
            refs_lock.lock();
            sensors_lock.lock();
            if(ref[1] < desiered_angle + angle_range && ref[1] > desiered_angle - angle_range){
                if(targets == 1){
                    // state = GO_TO_SECOND;
                    state = END;
                    // Where the robot is: (40,20); where it needs to go: (30,2)
                    ref[1] = angle_from_positions(  targets_position[0],
                                                    targets_position[1],
                                                    targets_position[2],
                                                    targets_position[3]) - readings[1];
                }
                else{
                    state = GO_TO_LAST;
                    // Where the robot is: (30,2); where it needs to go: (6,18)
                    ref[1] = angle_from_positions(  targets_position[2],
                                                    targets_position[3],
                                                    targets_position[4],
                                                    targets_position[5]) - readings[1];
                }
            }
            else{
                if(ref[1] > desiered_angle)
                    ref[1] -= ANGLE_TO_ADD;
                else
                    ref[1] += ANGLE_TO_ADD / 2;
            }
            refs_lock.unlock();
            sensors_lock.unlock();
            break;

        case DODGE:
            refs_lock.lock();
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
                sensors_lock.lock();
                state = GO_TO_LAST;
                // Where the robot is: (30,2); where it needs to go: (6,18)
                ref[1] = angle_from_positions(  robot_position[0],
                                                robot_position[1],
                                                targets_position[4],
                                                targets_position[5])  - readings[1];
                sensors_lock.unlock();
            }
            refs_lock.unlock();
            update_robot_position(readings[0], readings[1]);
            break;
            
        case END:
            refs_lock.lock();
            ref[0] = STOP;
            refs_lock.unlock();
            break;
        
        default:
            state = END;
            break;
        }

        
        readings[0] = 0;

        rc_usleep(NAVIGATION_PERIOD*1000);
    }
}