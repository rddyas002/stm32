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
#define ARM_MATH_CM4
#include "arm_math.h"

__IO uint32_t SysTickCounter;
void SysTick_Handler(void){
	SysTickCounter++;
}

int main(void) {
	// enable FPU full access
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

	init_I2C1(); // initialize I2C peripheral
	init_gyro(I2C1);
	init_accel(I2C1);
	init_mag(I2C1);
	init_gpio();
	init_USART2(115200);
	init_cci(129);
	SysTick_Config(SystemCoreClock/1000);

	int16_t gyro[3] = { 0 };
	int16_t accel[3] = { 0 };
	int16_t mag[3] = { 0 };
	uint8_t buffer[128];// = {'h','e','l','l','o','\0','\0','\0'};
	while (1) {
		float gyro_f[3] = { 0 };
		float accel_f[3] = { 0 };
		float mag_f[3] = { 0 };
		read_gyro(I2C1, gyro);
		read_accel(I2C1, accel);
		float B_mag = read_mag(I2C1, mag_f)*1e-1;	// uT

		gyro_f[0] = gyro[0] * 70E-3f;
		gyro_f[1] = gyro[1] * 70E-3f;
		gyro_f[2] = gyro[2] * 70E-3f;
		accel_f[0] = accel[0] * 3.9E-3f;
		accel_f[1] = accel[1] * 3.9E-3f;
		accel_f[2] = accel[2] * 3.9E-3f;
		sprintf(&buffer[0], "%7lu%7.1f%7.1f%7.1f%6.1f%6.1f%6.1f%7.1f%7.1f%7.1f%7.1fuT\r\n",
				SysTickCounter,
				gyro_f[0], gyro_f[1], gyro_f[2],
				accel_f[0], accel_f[1], accel_f[2],
				mag_f[0], mag_f[1], mag_f[2], B_mag);
		USART_puts(USART2, &buffer[0]);
		//SendMessageCan(129, 128, 1, CCI_MESSAGETYPE_EVENT, &buffer[0], 8);
		Delay(0x3FFFFF);
	}
}
