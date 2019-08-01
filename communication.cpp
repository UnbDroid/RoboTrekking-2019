#include "communication.hpp"

void* send_pwm(int pwmR, int pwmL){
	int arduino_bus = 1; // Bus to communicate with Arduino
    char str[10] = {'d', '+', '0', '0', '0', 'e', '+', '0', '0', '0'};
    uint8_t leftMotor, rightMotor;

    if(pwmR < 0){
        str[1] = '-'
        pwmR *= -1;
    }
    if(pwmL < 0){
        str[6] = '-'
        pwmL *= -1;
    }

    // Casting
    uint8_t pwmsR = (uint8_t)pwmR;
    uint8_t pwmsL = (uint8_t)pwmL;

    if(rc_enable_signal_handler() == -1){
        cout << "Could not start signal handler!" << endl;
        return NULL;
    }

	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if (rc_uart_init(arduino_bus, BAUDRATE, TIMEOUT_S, 0, 1, 0)){
		cout << "Failed to rc_uart_init" << endl;
		return NULL;
	}

    while(1){
        leftMotor = pwmsL;
        rightMotor = pwmsR;

        str[2] = '0' + (leftMotor/100);
        str[3] = '0' + (leftMotor%100)/10;
        str[4] = '0' + (leftMotor%10);
        str[7] = '0' + (rightMotor/100);
        str[8] = '0' + (rightMotor%100)/10;
        str[9] = '0' + (rightMotor%10);

        rc_uart_flush(arduino_bus); // Flush because we do not want trash into the communication line

        rc_uart_write(arduino_bus, (uint8_t *)str, 8);

        if(rc_get_state() == EXITING)
            break;
    }

	// close cleanly
	rc_uart_close(arduino_bus);
	return NULL;
}