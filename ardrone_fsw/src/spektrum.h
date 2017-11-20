
#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_

#include <string.h>
#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdbool.h>

#define BAUD B115200
#define DATABITS CS8
#define STOPBITS 0
#define PARITY 0
#define PARITYON 0
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1         //POSIX compliant source
#define SPEKTRUM_RX_LOGFILE "spektrum_log.csv"

#define SPEKTRUMRX_m0 (1.1721)
#define SPEKTRUMRX_c0 (940)
#define SPEKTRUM2PULSEWIDTH(x) ((unsigned short int) (SPEKTRUMRX_m0*(float)x + SPEKTRUMRX_c0))

// Check bits
#define SPEKTRUM_CH0 (0b00000000)
#define SPEKTRUM_CH1 (0b00000100)
#define SPEKTRUM_CH2 (0b00001000)
#define SPEKTRUM_CH3 (0b00001000)
#define SPEKTRUM_CH4 (0b00010000)
#define SPEKTRUM_CH5 (0b00010100)
#define SPEKTRUM_CH6 (0b00011000)

#define SPEKTRUM_LOGGING

bool spektrum_init(void);
void close_spektrum(void);
void printBuffer(void);
float getThrottle(void);
float getRoll(void);
float getPitch(void);
float getYaw(void);
float getAuto(void);
bool initOkay(void);

#endif /* SPEKTRUM_H_ */
