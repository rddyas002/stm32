#include <stdio.h>
#include <stdlib.h>
#define _POSIX_THREAD_SAFE_FUNCTIONS
#include <time.h>
#include <signal.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <windows.h>

static volatile int keepRunning = 1;
void intHandler(int dummy) {
    keepRunning = 0;
}

int openSerialComms(HANDLE hSerial){
	DCB dcbSerialParams = {0};
	COMMTIMEOUTS timeouts = {0};

	// Set device parameters (115200 baud, 1 start bit,
	// 1 stop bit, no parity)
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(hSerial, &dcbSerialParams) == 0){
		fprintf(stderr, "Error getting device state\n");
		CloseHandle(hSerial);
		return 1;
	}

	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if(SetCommState(hSerial, &dcbSerialParams) == 0){
		fprintf(stderr, "Error setting device parameters\n");
		CloseHandle(hSerial);
		return 1;
	}

	// Set COM port timeout settings
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if(SetCommTimeouts(hSerial, &timeouts) == 0){
		fprintf(stderr, "Error setting timeouts\n");
		CloseHandle(hSerial);
		return 1;
	}
	return 0;
}

int closeSerialComms(HANDLE hSerial){
	// Close serial port
	fprintf(stderr, "Closing serial port...");
	if (CloseHandle(hSerial) == 0)
	{
		fprintf(stderr, "Error\n");
		return 1;
	}
	return 0;
}

int sendSerialComms(HANDLE hSerial, char * data, char length){
	// Send specified text (remaining command line arguments)
	DWORD bytes_written;
	if(!WriteFile(hSerial, data, length, &bytes_written, NULL)){
		fprintf(stderr, "Error\n");
		CloseHandle(hSerial);
		return 1;
	}
	return 0;
}

DWORD readSerialComms(HANDLE hSerial, char * data){
	DWORD bytes_read;
	if(!ReadFile(hSerial, data, 11, &bytes_read, NULL)){
		fprintf(stderr, "Error\n");
		CloseHandle(hSerial);
		return 1;
	}
	return bytes_read;
}

int main(int argc, char *argv[]){
	signal(SIGINT, intHandler);

	char bytes_to_send[11] = {'0','1','2','3','4','5','6','7','8','9','\0'};

	// Declare variables and structures
	HANDLE hSerial;

	if (argc < 2){
		printf("Please provide COM port as argument.\r\n");
		return 1;
	}

	char * com_string = (char *) malloc(64);
	strcpy(com_string, "\\\\.\\");
	strcat(com_string, argv[1]);

	fprintf(stderr, "Opening serial port...");
	hSerial = CreateFile(com_string, GENERIC_READ|GENERIC_WRITE, 0, NULL,	OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	if (hSerial == INVALID_HANDLE_VALUE){
		fprintf(stderr, "Error\n");
		return 1;
	}
	else fprintf(stderr, "OK\n");

	openSerialComms(hSerial);

	struct timespec transmit_time;
	struct timespec receive_time;
	char received_bytes[64];
	while(keepRunning){
		// send 16 bytes
		clock_gettime(CLOCK_MONOTONIC, &transmit_time);
		sendSerialComms(hSerial, &bytes_to_send[0], 11);
		// wait for data return
		DWORD len = readSerialComms(hSerial, &received_bytes[0]);
		clock_gettime(CLOCK_MONOTONIC, &receive_time);
		double delta_t = (receive_time.tv_sec - transmit_time.tv_sec) + (double)(receive_time.tv_nsec - transmit_time.tv_nsec)/1e9;
		printf("dt: %6.0fms\n", delta_t*1e3);
		sleep(1);
	}

	closeSerialComms(hSerial);

	// exit normally
	return 0;

}
