#include <windows.h>
#include <stdio.h>

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
	fprintf(stderr, "Sending bytes...");
	if(!WriteFile(hSerial, data, length, &bytes_written, NULL)){
		fprintf(stderr, "Error\n");
		CloseHandle(hSerial);
		return 1;
	}
	fprintf(stderr, "%lu bytes written\n", bytes_written);
	return 0;
}

int main(int argc, char *argv[]){
	// Define the five bytes to send ("hello")
	char bytes_to_send[5];
	bytes_to_send[0] = 104;
	bytes_to_send[1] = 101;
	bytes_to_send[2] = 108;
	bytes_to_send[3] = 108;
	bytes_to_send[4] = 111;

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
	sendSerialComms(hSerial, &bytes_to_send[0], 5);
	closeSerialComms(hSerial);

	// exit normally
	return 0;

}
