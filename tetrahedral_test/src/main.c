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
	init_ekf();
	SysTick_Config(SystemCoreClock/1000);

	imu_data_s imu_data;
	while (1) {
		int8_t temperature;
		read_gyro(I2C1, imu_data.rate, &temperature);
		read_accel(I2C1, imu_data.acceleration);
		read_mag(I2C1, imu_data.magnetic);	// uT
		imu_data.time = (float)SysTickCounter*1.0e-3f;
		USART_send(USART2, (uint8_t *) &imu_data, sizeof(imu_data_s));
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		msDelaySysTick(100);
	}
}
