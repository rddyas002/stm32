#include <stdio.h>
#include <math.h>
#include <string.h>

#include "stm32f4_discovery.h"
#include "i2c.h"
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "usart.h"
#include "gpio.h"
#include "CciProtocol.h"
#include "ekf.h"

__IO uint32_t SysTickCounter = 0;
__IO int32_t DelayCounter = 0;
void SysTick_Handler(void){
	SysTickCounter++;
	DelayCounter--;
}

void msDelaySysTick(uint32_t count_ms){
	DelayCounter = count_ms;
	while (DelayCounter > 0);
}

typedef struct{
	float rate[3];			// dps
	float acceleration[3];	// g
	float magnetic[3];		// uT
	float time;
}__attribute__((packed)) imu_data_s;

void computeInitMeasurementFrame(float32_t y[6]){
	int i;
	double measurements[6] = {0};
	imu_data_s imu_data;
	int8_t temperature;

	for (i = 0; i < 40; i++){
		read_accel(I2C1, imu_data.acceleration);
		read_mag(I2C1, imu_data.magnetic);
		// store data
		measurements[0] += (double)imu_data.acceleration[0];
		measurements[1] += (double)imu_data.acceleration[1];
		measurements[2] += (double)imu_data.acceleration[2];
		measurements[3] += (double)imu_data.magnetic[0];
		measurements[4] += (double)imu_data.magnetic[1];
		measurements[5] += (double)imu_data.magnetic[2];
		msDelaySysTick(50);
	}

	for (i = 0; i < 6; i++)
		measurements[i] /= 40.0;

	for (i = 0; i < 6; i++)
		y[i] = (float32_t)measurements[i];

}

int main(void) {
	float32_t average_imu[6] = {0};
	float32_t w[3] = {0};
	float32_t q[4] = {0};
	// enable FPU full access
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

	SysTick_Config(SystemCoreClock/1000);
	init_I2C1(); // initialize I2C peripheral
	init_gyro(I2C1);
	init_accel(I2C1);
	init_mag(I2C1);
	init_gpio();
	init_USART2(115200);
	init_cci(129);
	computeInitMeasurementFrame(average_imu);
	init_ekf(average_imu);

	imu_data_s imu_data;
	float32_t ypr[3] = {0};
	char buffer[128];
	while (1) {
		int8_t temperature;
		read_gyro(I2C1, imu_data.rate, &temperature);
		read_accel(I2C1, imu_data.acceleration);
		read_mag(I2C1, imu_data.magnetic);	// uT
		imu_data.time = (float)SysTickCounter*1.0e-3f;
		run_ekf(0.1, imu_data.rate, imu_data.acceleration, imu_data.magnetic, &q[0], &w[0]);
		imu_data.time = (float)SysTickCounter*1.0e-3f;
		ypr[0] = atan2(2.0f*q[2]*q[3] + 2.0f*q[0]*q[1], q[3]*q[3] - q[2]*q[2] - q[1]*q[1] + q[0]*q[0])*180.0f/M_PI_f;
		ypr[1] = -asin(2.0f*q[1]*q[3] - 2.0f*q[0]*q[2])*180.0f/M_PI_f;
		ypr[2] = atan2(2.0f*q[1]*q[2] + 2.0f*q[0]*q[3], q[1]*q[1] + q[0]*q[0] - q[3]*q[3] - q[2]*q[2])*180.0f/M_PI_f;
		//q2ypr(q, ypr);
		//sprintf(&buffer[0], "%5.2f,%5.2f,%5.2f,%5.2f|%5.2f,%5.2f,%5.2f\r\n", q[0], q[1], q[2], q[3], w[0]*180/M_PI, w[1]*180/M_PI, w[2]*180/M_PI);
		sprintf(&buffer[0], "%5.2f,%5.2f,%5.2f|%5.2f,%5.2f,%5.2f\r\n", ypr[0], ypr[1], ypr[2], w[0]*180/M_PI, w[1]*180/M_PI, w[2]*180/M_PI);
		USART_puts(USART2, &buffer[0]);
		//USART_send(USART2, (uint8_t *) &imu_data, sizeof(imu_data_s));
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		msDelaySysTick(100);
	}
}
