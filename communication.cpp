#include "communication.hpp"

void* send_pwm(void *pwm){
	int arduino_bus = 1; // Bus to communicate with Arduino

    // Casting
    uint8_t* pwms = (uint8_t*)pwm;

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
        uint8_t leftMotor = pwms[0];
        uint8_t rightMotor = pwms[1];

        if (rightMotor == uint8_t('d') || rightMotor == uint8_t('e'))
            rightMotor -= 1;
        if (leftMotor == uint8_t('d') || leftMotor == uint8_t('e'))
            leftMotor -= 1;

        char str[4] = {'d', char(rightMotor), 'e', char(leftMotor)};

        rc_uart_flush(arduino_bus); // Flush because we do not want trash into the communication line

        rc_uart_write(arduino_bus, (uint8_t *)str, strlen(str));
    
        if(rc_get_state() == EXITING)
            break;
    }

	// close cleanly
	rc_uart_close(arduino_bus);
	return NULL;
}