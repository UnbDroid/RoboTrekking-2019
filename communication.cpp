#include "communication.h"

void* send_pwm(void *args){
	int arduino_bus = 1; // Bus to communicate with Arduino
    char str[8] = {'*', '0', '0', '0', '*', '0', '0', '0'}; // The ones left with '*' can be either a letter or a number
    uint8_t leftMotor, rightMotor;
    uint8_t us_data;

    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if (rc_uart_init(arduino_bus, BAUDRATE, TIMEOUT_S, 0, 1, 0)){
		return NULL;
	}

    // Casting
    commArgs* comm_args = (commArgs*)args;
    uint8_t* pwms = comm_args->arg_pwms;
    bool* which_us = comm_args->which_us;
    bool* flag = comm_args->flag;

    mutex* nav_mutex = comm_args->navigation_mutex;

    unique_lock<mutex> nav_lock(*nav_mutex, defer_lock);

    // Start string to send, the one that is zero is the first to be sent
    str[0] = 'e'*((INDEX_LEFT + 1)%2) + 'd'*((INDEX_RIGHT + 1)%2);
    str[4] = 'e'*INDEX_LEFT + 'd'*INDEX_RIGHT;

    for(;;){
        leftMotor = pwms[INDEX_LEFT];
        rightMotor = pwms[INDEX_RIGHT];

        str[4*INDEX_LEFT + 1] = '0' + (leftMotor/100);
        str[4*INDEX_LEFT + 3] = '0' + (leftMotor%10);
        str[4*INDEX_LEFT + 2] = '0' + (leftMotor%100)/10;
        str[4*INDEX_RIGHT + 1] = '0' + (rightMotor/100);
        str[4*INDEX_RIGHT + 2] = '0' + (rightMotor%100)/10;
        str[4*INDEX_RIGHT + 3] = '0' + (rightMotor%10);

        // Remember to reset this variable state in the navigation
        if(rc_uart_bytes_available(arduino_bus) && (*flag)) {
            nav_lock.lock();

            rc_uart_read_bytes(arduino_bus, &us_data, 1);
            if(us_data & 0x01) which_us[0] = true;
            if(us_data & 0x02) which_us[1] = true;
            if(us_data & 0x04) which_us[2] = true;
            if(us_data & 0x08) which_us[3] = true;
        
            nav_lock.unlock();
        }

        rc_uart_flush(arduino_bus); // Flush because we do not want trash into the communication line

        rc_uart_write(arduino_bus, (uint8_t *)str, 8);

        if(rc_get_state() == EXITING)
            break;
    }

	// Close cleanly
	rc_uart_close(arduino_bus);
	return NULL;
}