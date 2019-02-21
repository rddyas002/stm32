/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#include "i2c.h"
#include "hmc5883l.h"
#include "MPU6050.h"
#include "ekf32.h"

#include <arm_math.h>

xTimerHandle sampleIMUHandler;

void ToggleLED_Timer(void*);
void SampleIMU(xTimerHandle pxTimer);
void initHW();
void computeGyroStats(imu_data_s * imu_data);

xQueueHandle pbq;

void computeInitMeasurementFrame(imu_data_s * imu_data) {
	int i;
	long int wait = 0;
	double measurements[9] = { 0 };

	for (i = 0; i < 50; i++) {
		MPU6050_GetRawAccelGyro(imu_data);
		read_mag(I2C1, imu_data->magnetic);
		computeGyroStats(imu_data);
		// store data
		measurements[0] += (double) imu_data->acceleration[0];
		measurements[1] += (double) imu_data->acceleration[1];
		measurements[2] += (double) imu_data->acceleration[2];
		measurements[3] += (double) imu_data->rate[0];
		measurements[4] += (double) imu_data->rate[1];
		measurements[5] += (double) imu_data->rate[2];
		measurements[6] += (double) imu_data->magnetic[0];
		measurements[7] += (double) imu_data->magnetic[1];
		measurements[8] += (double) imu_data->magnetic[2];
		for (wait = 0; wait < 10000; wait++);
	}

	for (i = 0; i < 9; i++)
		measurements[i] /= 50.0;

	imu_data->accel_offset[0] = (float32_t) measurements[0];
	imu_data->accel_offset[1] = (float32_t) measurements[1];
	imu_data->accel_offset[2] = (float32_t) measurements[2];

	imu_data->gyro_offset[0] = (float32_t) measurements[3];
	imu_data->gyro_offset[1] = (float32_t) measurements[4];
	imu_data->gyro_offset[2] = (float32_t) measurements[5];

	imu_data->mag_offset[0] = (float32_t) measurements[6];
	imu_data->mag_offset[1] = (float32_t) measurements[7];
	imu_data->mag_offset[2] = (float32_t) measurements[8];

}

float32_t computeMean(float32_t m, float32_t x, float32_t k) {
	return m + (x - m) / k;
}

float32_t computeVariance(float32_t v, float32_t x, float32_t m, float32_t m_k) {
	return v + (x - m) * (x - m_k);
}

void computeGyroStats(imu_data_s * imu_data) {
	static float32_t m_g[3] = { 0 };
	static float32_t v_g[3] = { 0 };
	static float32_t m_a[3] = { 0 };
	static float32_t v_a[3] = { 0 };
	static float32_t m_m[3] = { 0 };
	static float32_t v_m[3] = { 0 };

	static uint32_t count = 0;
	static bool first_enter = true;
	int i;

	if (first_enter) {
		for (i = 0; i < 3; i++) {
			v_g[i] = 0.0f;
			m_g[i] = imu_data->rate[i];
			v_a[i] = 0.0f;
			m_a[i] = imu_data->acceleration[i];
			v_m[i] = 0.0f;
			m_m[i] = imu_data->magnetic[i];
		}

		count++;
		first_enter = false;
		return;
	}

	float32_t m_k;
	for (i = 0; i < 3; i++) {
		m_k = computeMean(m_g[i], imu_data->rate[i], (float32_t) count);
		v_g[i] = computeVariance(v_g[i], imu_data->rate[i], m_g[i], m_k);
		m_g[i] = m_k;

		m_k = computeMean(m_a[i], imu_data->acceleration[i], (float32_t) count);
		v_a[i] = computeVariance(v_a[i], imu_data->acceleration[i], m_a[i],
				m_k);
		m_a[i] = m_k;

		m_k = computeMean(m_m[i], imu_data->magnetic[i], (float32_t) count);
		v_m[i] = computeVariance(v_m[i], imu_data->magnetic[i], m_m[i], m_k);
		m_m[i] = m_k;

		count++;
		imu_data->gyro_var[i] = v_g[i] / ((float32_t) count - 1.0f);
		imu_data->accel_var[i] = v_a[i] / ((float32_t) count - 1.0f);
		imu_data->mag_var[i] = v_m[i] / ((float32_t) count - 1.0f);
	}
}

int main(void){
	imu_data_s imu_data;

	float32_t Q_diag[6] = { 0.35e-5f, 0.35e-5f, 0.35e-5f, 2.5e-8f, 2.5e-8f,
			2.5e-8f };
	float32_t R_diag[6] =
			{ 0.3e-4f, 0.3e-4f, 0.3e-4f, 0.4e-4f, 0.4e-4f, 0.4e-4f };

	float32_t b_32[3] = { 0 };
	float32_t q_32[4] = { 0 };
	float32_t ypr_32[3] = { 0 };
	float64_t b_64[3] = { 0 };
	float64_t q_64[4] = { 0 };
	float64_t ypr_64[3] = { 0 };

	// enable FPU full access
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

	initHW();

	init_ekf_32(&imu_data);
	computeInitMeasurementFrame(&imu_data);

	/* Create IPC variables */
	pbq = xQueueCreate(10, sizeof(int));
	if (pbq == 0) {
		while(1); /* fatal error */
	}

	/* Create tasks */
	xTaskCreate(
			ToggleLED_Timer,                 /* Function pointer */
			"Task1",                          /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			NULL                              /* Task handle */
	);

	sampleIMUHandler = xTimerCreate(
			"imuTimer",
	        pdMS_TO_TICKS(1000),
	        pdTRUE,
	        (void*)0,
			SampleIMU);

	if (sampleIMUHandler == NULL) {
		while(1);
	}
	else
		xTimerStart(sampleIMUHandler, 0);

	/* Start the RTOS Scheduler */
	vTaskStartScheduler();

	/* HALT */
	while(1);
}

void ToggleLED_Timer(void *pvParameters){
	int counter = 0;
	int sig;
	while (1) {
		/*
		portBASE_TYPE status = xQueueReceive(pbq, &sig, 0);
    if(status == pdTRUE) {
    	if(counter++ > 3) counter = 0;
    }

    switch (counter){
    case 0:
        GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    	break;
    case 1:
        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    	break;
    case 2:
        GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    	break;
    case 3:
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
    	break;
    default:
        GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    	break;
    }
*/
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void SampleIMU(xTimerHandle pxTimer){
	float imu[6];
	float mag[3];

	// read sensors
	MPU6050_GetRawAccelGyro(imu);
	read_mag(I2C1, mag);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
}

void initHW(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure2;

	// Init LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Init PushButton
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure2);

	init_I2C1();
	MPU6050_Initialize();
	init_mag(I2C1);
}
