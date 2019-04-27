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

#include <stdio.h>
#include <string.h>
#include <rc/uart.h>

#define BUF_SIZE	32
#define TIMEOUT_S	0.5
#define BAUDRATE	9600
#define MAX_SIZE	1
#define ANSWER_SIZE	2

int main(){
	int arduino_bus = 1; // Bus to communicate with Arduino
	
	char* str[MAX_SIZE]; // Message to Arduino

	uint8_t buf[BUF_SIZE+1];
	int ret; // Number of bytes read off the arduino

	printf("\ntesting UART bus %d\n\n", arduino_bus);
	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(bus, BAUDRATE, TIMEOUT_S, 0,1,0)){
		printf("Failed to rc_uart_init%d\n", arduino_bus);
		return -1;
	}

	str[0] = '0'; // Start sending message as a zero
	rc_uart_flush(arduino_bus); // Flush because we do not want trash into the communication line

	// Communicate FOREVER!!!!
	while(1){
		printf("Sending %d bytes: %s \n", bytes, str);
		rc_uart_write(arduino_bus, (uint8_t*)str, MAX_SIZE);

		printf("Now it's time to read the answer:\n");
		memset(buf, 0, sizeof(buf));
		ret = rc_uart_read_bytes(arduino_bus, buf, ANSWER_SIZE); // We expect an OK from the arduino
		if(ret<0){
			printf("Error reading Bus!\n");
		}
		else if(ret == 0){
			print("No answer from the arduino!\n");
		}
		else {
			print("Answer was: %s\n", buf);
		}
		
		if(str[0] == '9'){
			str[0] = '0';
		}
		else {
			str[0]++;
		}
	}

	// close
	rc_uart_close(bus);
	return 0;
}
