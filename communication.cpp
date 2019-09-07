#include "communication.h"

void* send_pwm(void *pwm){
	int arduino_bus = 1; // Bus to communicate with Arduino
    char str[8] = {'*', '0', '0', '0', '*', '0', '0', '0'}; // The ones left with '*' can be either a letter or a number
    uint8_t leftMotor, rightMotor;

    // Casting
    uint8_t* pwms = (uint8_t*)pwm;

    if(rc_enable_signal_handler() == -1){
        return NULL;
    }

	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if (rc_uart_init(arduino_bus, BAUDRATE, TIMEOUT_S, 0, 1, 0)){
		return NULL;
	}

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

        rc_uart_flush(arduino_bus); // Flush because we do not want trash into the communication line

        rc_uart_write(arduino_bus, (uint8_t *)str, 8);

        if(rc_get_state() == EXITING)
            break;
    }

	// Close cleanly
	rc_uart_close(arduino_bus);
	return NULL;
}