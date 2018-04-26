/*
 * gpio.c
 *
 *  Created on: 06 Nov 2017
 *      Author: yreddi
 */

#include "gpio.h"

void init_gpio(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void init_tim4(void){
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable clock for TIM3 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_BaseStruct.TIM_Prescaler = 20;						// divide 84MHz clock by 21 to get to 4MHz
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV2;		// divide the 4MHz clock by 2 to get to 2MHz
	TIM_BaseStruct.TIM_Period = 39999; /* 50Hz PWM */
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM3 */
	TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
	/* Start count on TIM3 */
	TIM_Cmd(TIM3, ENABLE);


    /* PWM mode 2 = Clear on compare match */
    /* PWM mode 1 = Set on compare match */
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    // each clock represents 0.5us
    // min = 1148us/0.5 = 2296
    // neutral = 1488us/0.5 = 2976
    // max = 1832us/0.5 = 3664

    TIM_OCStruct.TIM_Pulse = PWM_ZERO;
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OCStruct.TIM_Pulse = PWM_ZERO;
    TIM_OC2Init(TIM3, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OCStruct.TIM_Pulse = PWM_ZERO;
    TIM_OC3Init(TIM3, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OCStruct.TIM_Pulse = PWM_ZERO;
    TIM_OC4Init(TIM3, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // enable io
    /* Clock for GPIOB */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}
