#include <stdio.h>
#include <stdlib.h>
#define _POSIX_THREAD_SAFE_FUNCTIONS
#include <time.h>
#include <signal.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <windows.h>
#include <stdint.h>

#include "ekf.h"

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
	timeouts.ReadIntervalTimeout = 500;
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

DWORD readSerialComms(HANDLE hSerial, uint8_t * data, uint16_t length){
	DWORD bytes_read;
	if(!ReadFile(hSerial, data, length, &bytes_read, NULL)){
		fprintf(stderr, "Error\n");
		CloseHandle(hSerial);
		return 1;
	}
	return bytes_read;
}

int main(int argc, char *argv[]){
	signal(SIGINT, intHandler);
	/*
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
	uint8_t received_bytes[64];
	imu_data_s imu_data;
	int counter = 0;
	*/
	imu_data_s imu_data;

	FILE *ifp, *ofp;
	//ifp = fopen("C:\\Users\\RDDYA\\Documents\\stm32\\serial_comms\\Debug\\imu.csv", "r");
	ifp = fopen("C:\\work\\stm32\\serial_comms\\Debug\\imu.csv", "r");
	ofp = fopen("C:\\work\\stm32\\serial_comms\\Debug\\imu_out.csv", "w");

	if (ifp == NULL) {
	  fprintf(stderr, "Can't open input file in.list!\n");
	  exit(1);
	}

	double delta_t = 0;
	double prev_t = 0;

	int i;
	double measurements[9] = {0};

	for (i = 0; i < 50; i++){
		if(fscanf(ifp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", &imu_data.time,
				&imu_data.rate[0],&imu_data.rate[1],&imu_data.rate[2],
				&imu_data.acceleration[0],&imu_data.acceleration[1],&imu_data.acceleration[2],
				&imu_data.magnetic[0],&imu_data.magnetic[1],&imu_data.magnetic[2]) == EOF){
			exit(1);
		}

		// store data
		measurements[0] += (double)imu_data.acceleration[0];
		measurements[1] += (double)imu_data.acceleration[1];
		measurements[2] += (double)imu_data.acceleration[2];
		measurements[3] += (double)imu_data.rate[0];
		measurements[4] += (double)imu_data.rate[1];
		measurements[5] += (double)imu_data.rate[2];
		measurements[6] += (double)imu_data.magnetic[0];
		measurements[7] += (double)imu_data.magnetic[1];
		measurements[8] += (double)imu_data.magnetic[2];
	}

	for (i = 0; i < 9; i++)
		measurements[i] /= 50.0;

	imu_data.accel_offset[0] = (float)measurements[0];
	imu_data.accel_offset[1] = (float)measurements[1];
	imu_data.accel_offset[2] = (float)measurements[2];

	imu_data.gyro_offset[0] = (float)measurements[3];
	imu_data.gyro_offset[1] = (float)measurements[4];
	imu_data.gyro_offset[2] = (float)measurements[5];

	imu_data.mag_offset[0] = (float)measurements[6];
	imu_data.mag_offset[1] = (float)measurements[7];
	imu_data.mag_offset[2] = (float)measurements[8];

	init_ekf(&imu_data);

	while(keepRunning){
/*		PurgeComm(hSerial, PURGE_RXCLEAR|PURGE_TXCLEAR);
		DWORD len = readSerialComms(hSerial, &received_bytes[0], 40);
		memcpy(&imu_data, &received_bytes[0], 40);
		printf("%7.3f:%6.1f %6.1f %6.1f|%6.1f %6.1f %6.1f\r\n",
					imu_data.time,
					imu_data.rate[0], imu_data.rate[1], imu_data.rate[2],
					imu_data.magnetic[0], imu_data.magnetic[1], imu_data.magnetic[2]);
					*/
		if(fscanf(ifp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", &imu_data.time,
				&imu_data.rate[0],&imu_data.rate[1],&imu_data.rate[2],
				&imu_data.acceleration[0],&imu_data.acceleration[1],&imu_data.acceleration[2],
				&imu_data.magnetic[0],&imu_data.magnetic[1],&imu_data.magnetic[2]) == EOF){
			break;
		}

		double q[4],b[3];
//		imu_data.rate[0] = -0.032;
//		imu_data.rate[1] = 0.022;
//		imu_data.rate[2] = -0.011;
//		imu_data.acceleration[0] = 0.001;
//		imu_data.acceleration[1] = 0.015;
//		imu_data.acceleration[2] = -1.0;
//		imu_data.magnetic[0] = 0.124;
//		imu_data.magnetic[1] = -0.658;
//		imu_data.magnetic[2] = 0.743;

		if (delta_t == 0){
			delta_t = 20e-3;
		}
		else{
			delta_t = (double)(imu_data.time - prev_t);
		}
		prev_t = imu_data.time;
		bool stop = false;
		if (imu_data.time > 20.7)
			stop = true;
		run_ekf(delta_t, imu_data.rate, imu_data.acceleration, imu_data.magnetic, &q[0], &b[0]);
		double ypr[3] = {0};
		q2ypr(q, ypr);
		fprintf(ofp,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", q[0], q[1], q[2], q[3], b[0], b[1], b[2], ypr[0], ypr[1], ypr[2]);
	}

	fclose(ifp);
	fclose(ofp);
	//closeSerialComms(hSerial);

	// exit normally
	return 0;

}
