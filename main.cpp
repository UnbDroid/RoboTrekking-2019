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
#include <signal.h>

using namespace std;

extern "C"
{
#include <rc/uart.h>
#include <rc/start_stop.h>
#include <rc/button.h>
}

#define BUF_SIZE 32
#define TIMEOUT_S 0.5
#define BAUDRATE 115200
#define MAX_SIZE 1
#define ANSWER_SIZE 2
#define PKG_SIZE

#define RIGHT_MOTOR 35
#define LEFT_MOTOR 45

static int running = 0;
bool started = false;
int platform = 0;

enum State{
	FORWARD,
	SWERVE,
	FIX_ROUTE,
	AROUND_CONE,
	END
}

static void starting_code(void)
{
        std::cout << "Starting code" << std::endl;
		started = true;
        return;
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

int main()
{
	if (rc_kill_existing_process(2.0) < -2)
		return -1;
	if (rc_enable_signal_handler() == -1)
	{
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}
	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;
	State state = FORWARD;

	rc_make_pid_file();
	
	rc_button_set_callbacks(RC_BTN_PIN_MODE, starting_code, NULL);
	while(started && running && state != END){
		// calculate track to the desered point
	
		// verify if robot is on the right track 


	}
	// cleanup and exit
	rc_button_cleanup();
	return 0;
}