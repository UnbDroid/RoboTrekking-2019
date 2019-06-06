/**
 * @file rc_uart.c
 *
 * This will be the file responsible for dealing with the communication between the BBB and 
 * an arduino using a UART bus.
 * 
 * connect the RX and TX wires of one of the included 4-pin
 * JST-SH pigtails and plug into the UART1 or UART5 headers. You may also elect
 * to test UART0 on the debug header or UART2 on the GPS header. The test
 * strings this programs transmits will then loopback to the RX channel.
 */

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>

using namespace std;

extern "C"
{
#include <rc/uart.h>
#include <rc/start_stop.h>
}

#define BUF_SIZE 32
#define TIMEOUT_S 0.5
#define BAUDRATE 115200
#define MAX_SIZE 1
#define ANSWER_SIZE 2
#define PKG_SIZE

#define RIGHT_MOTOR 35
#define LEFT_MOTOR 45

int main()
{
	if (rc_kill_existing_process(2.0) < -2)
		return -1;
	if (rc_enable_signal_handler() == -1)
	{
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return -1;
	}
	rc_make_pid_file();

	int arduino_bus = 2; // Bus to communicate with Arduino

	// string str = "0"; // Message to Arduino

	uint8_t rightMotor = RIGHT_MOTOR;
	uint8_t leftMotor = LEFT_MOTOR;

	if (rightMotor == 'd' || rightMotor == 'e')
		rightMotor -= 1;
	if (leftMotor == 'd' || leftMotor == 'e')
		leftMotor -= 1;

	char str[4] = {'d', rightMotor, 'e', leftMotor};

	uint8_t buf[BUF_SIZE + 1];
	int ret; // Number of bytes read off the arduino

	printf("\ntesting UART bus %d\n\n", arduino_bus);
	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if (rc_uart_init(arduino_bus, BAUDRATE, TIMEOUT_S, 0, 1, 0))
	{
		printf("Failed to rc_uart_init%d\n", arduino_bus);
		return -1;
	}
	printf("rc_uart_init%d inicialized\n", arduino_bus);

	// *str[0] = '0';				// Start sending message as a zero
	rc_uart_flush(arduino_bus); // Flush because we do not want trash into the communication line

	// Communicate FOREVER!!!!
	printf("Starting...\n");
	while (1)
	{
		printf("Sending %d bytes: %s \n", strlen(str), str);
		rc_uart_write(arduino_bus, (uint8_t *)str, strlen(str));

		printf("Now it's time to read the answer:\n");
		memset(buf, 0, sizeof(buf));
		ret = rc_uart_read_bytes(arduino_bus, buf, ANSWER_SIZE); // We expect an OK from the arduino
		if (ret < 0)
		{
			printf("Error reading Bus!\n");
		}
		else if (ret == 0)
		{
			printf("Not compatible string!\n");
		}
		else if (ret == 1)
		{
			printf("Compatible string!\n");
		}
		else
		{
			printf("Answer was: %s\n", buf);
		}
	}

	// close
	rc_uart_close(arduino_bus);
	rc_remove_pid_file();
	return 0;
}
