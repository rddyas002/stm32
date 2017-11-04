/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "i2c.h"
#include "l3g4200d.h"
#include "usart.h"
#include <math.h>

void Delay(__IO uint32_t nCount);

void init_gpio(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;// | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

int main(void) {
	// enable FPU full access
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); /* set CP10 and CP11 Full Access */

	init_I2C1(); // initialize I2C peripheral
	init_gyro(I2C1);
	init_gpio();
	init_USART1(115200);

	int16_t gyro[3] = { 0 };
	float gyro_f[3] = { 0 };
	while (1) {
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		read_gyro(I2C1, gyro);
		gyro_f[0] = gyro[0] * 70E-3f;
		gyro_f[1] = gyro[1] * 70E-3f;
		gyro_f[2] = gyro[2] * 70E-3f;
		Delay(0x3FFFFF);
	}
}