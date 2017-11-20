#include "spektrum.h"

int spektrum_fd;
struct termios options;       	//place for old and new port settings for serial port
struct sigaction saio;          //definition of signal action
char read_buffer[64];           //buffer for where data is put
int read_len;
char devicename[50];
double time_t0;

pthread_t spektrum_thread;
pthread_mutex_t count_mutex;

unsigned short int channel[6];
float reference_command[6];

void decodePacket(char bytes);
float deadband(float reference, float half_width);

unsigned short int channel[6];
char write_buffer[128];

static void *spektrum_read(void *data __attribute__((unused)))
{
    printf("Spektrum receive thread started.\n");

    // Flush read buffer
    tcflush(spektrum_fd,TCIFLUSH);

	while(1){
        int bytes_available = 0;
        ioctl(spektrum_fd, FIONREAD, &bytes_available);
        if (bytes_available > 14){
            int n = read(spektrum_fd,&read_buffer[0], 64);
            read_buffer[n] = '\0';
            pthread_mutex_lock(&count_mutex);
            decodePacket(n);
            pthread_mutex_unlock(&count_mutex);
        }
        usleep(10000);
	}
	return NULL;
}

bool spektrum_init(void) {
    sprintf(devicename, "%s", MODEMDEVICE);
    spektrum_fd = open(devicename, O_RDWR | O_NOCTTY);
    if (spektrum_fd < 0){
        perror(devicename);
        return false;
    }
    else{
    	printf("Port %s successfully opened.\n", devicename);
    }

    tcgetattr(spektrum_fd, &options);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);
    // set baud rate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    tcsetattr(spektrum_fd, TCSANOW, &options);

	if (pthread_create(&spektrum_thread, NULL, spektrum_read, NULL) != 0) {
		printf("[spektrum] Could not create spektrum reading thread!\n");
		return false;
	}

	return true;
}

void close_spektrum(void){
	close(spektrum_fd);
}

void decodePacket(char bytes){
    unsigned short int spektrum_word;
    short int temp = 0;

    // rough error checking
	if ((read_buffer[0] == 0x03) && (read_buffer[1] == 0x01) && (read_buffer[bytes-2] == 0x18)){
        if ((read_buffer[2] & 0xFC) == SPEKTRUM_CH1){
            spektrum_word = ((unsigned short int)read_buffer[2] << 8) & 0x03FF;
            channel[1] = (unsigned short int) (spektrum_word | (unsigned char)read_buffer[3]);
            reference_command[1] = ((float)channel[1] - 511)/360;
        }
        if ((read_buffer[4] & 0xFC) == SPEKTRUM_CH5){
            spektrum_word = ((unsigned short int)read_buffer[4] << 8) & 0x03FF;
            channel[5] = (unsigned short int) (spektrum_word | (unsigned char)read_buffer[5]);
            if (channel[5] > 180){
            	reference_command[5] = 0;
            }
            else{
            	reference_command[5] = 1;
            }

        }
        if ((read_buffer[6] & 0xFC) == SPEKTRUM_CH2){
            spektrum_word = ((unsigned short int)read_buffer[6] << 8) & 0x03FF;
            channel[2] = (unsigned short int) (spektrum_word | (unsigned char)read_buffer[7]);
            reference_command[2] = -((float)channel[2] - 511)/360;
        }
        if ((read_buffer[8] & 0xF8) == SPEKTRUM_CH3){
            spektrum_word = ((unsigned short int)read_buffer[8] << 8) & 0x07FF;
            channel[3] = (unsigned short int) (spektrum_word | (unsigned char)read_buffer[9]);
            // +ve signal
            if ((channel[3] & (0b11 << 9)) == (0b11 << 9)){
            	temp = (short int)(channel[3] & 0x01FF);
            }
            else if ((channel[3] & (0b01 << 9)) == (0b01 << 9)){// -ve signal
            	temp = (signed short int) (-255 + (channel[3] & 0xFF));
            }
            else if ((channel[3] & (0b10 << 9)) == (0b10 << 9)){
            	temp = -255 + (signed short int) (-255 + (channel[3] & 0xFF));
            }
            reference_command[3] = -(float)temp/360;
        }
        if ((read_buffer[10] & 0xFC) == SPEKTRUM_CH0){
            spektrum_word = ((unsigned short int)read_buffer[10] << 8) & 0x03FF;
            channel[0] = (unsigned short int) (spektrum_word | (unsigned char)read_buffer[11]);
            reference_command[0] = (float)channel[0]/400 - 1.06;
        }
        if (bytes == 16){
            	reference_command[4] = 0;
        }
        else{
        	reference_command[4] = 1;

        }
    }
}

bool initOkay(void){
	// check if all sticks are centered and land selected all is okay
	if (getThrottle() == 0){
		if (getRoll() == 0){
			if (getPitch() == 0){
				if (getYaw() == 0){
					if (getAuto() == 0)
						return true;
					else
						return false;
				}
				else
					return false;
			}
			else
				return false;
		}
		else
			return false;
	}
	else
		return false;
}

float deadband(float reference, float half_width){
    if ((reference > -half_width) && (reference < half_width)){
    	reference = 0;
    }
    else{
    	if (reference > half_width)
    		reference -= half_width;
    	if (reference < -half_width)
    		reference += half_width;
    }
    return reference;
}

float getThrottle(void){
	return deadband(reference_command[0], 0.2);
}

float getRoll(void){
	return deadband(reference_command[1], 0.05);
}

float getPitch(void){
	return deadband(reference_command[2], 0.05);
}

float getYaw(void){
	return deadband(reference_command[3], 0.05);
}

float getAuto(void){
	return reference_command[5];
}
