#include <stdio.h>
#include <math.h>
#include <string.h>

#include "stm32f4_discovery.h"
#include "i2c.h"
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "MPU6050.h"
#include "usart.h"
#include "gpio.h"
#include "CciProtocol.h"
#include "ekf.h"

float32_t computeMean(float32_t m, float32_t x, float32_t k);
float32_t computeVariance(float32_t v, float32_t x, float32_t m, float32_t m_k);
void computeGyroStats(float32_t gyro[3], float32_t accel[3], float32_t mag[3], float32_t gyro_var[3], float32_t accel_var[3], float32_t mag_var[3]);

__IO uint32_t SysTickCounter = 0;
__IO int32_t DelayCounter = 0;
__IO bool flag10ms = false;
__IO bool flag100ms = false;
void timingHandler(void){
	SysTickCounter++;

	if(!(SysTickCounter % 20))
		flag10ms = true;
	if(!(SysTickCounter % 100))
		flag100ms = true;

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
	float32_t gyro_var[3] = {0};
	float32_t accel_var[3] = {0};
	float32_t mag_var[3] = {0};
	imu_data_s imu_data;
	int8_t temperature;

	for (i = 0; i < 50; i++){
		read_gyro(I2C1, imu_data.rate, &temperature);
		read_accel(I2C1, imu_data.acceleration);
		read_mag(I2C1, imu_data.magnetic);
		computeGyroStats(imu_data.rate, imu_data.acceleration, imu_data.magnetic, gyro_var, accel_var, mag_var);
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
		measurements[i] /= 50.0;

	for (i = 0; i < 6; i++)
		y[i] = (float32_t)measurements[i];

}

float32_t computeMean(float32_t m, float32_t x, float32_t k){
	return m + (x - m)/k;
}

float32_t computeVariance(float32_t v, float32_t x, float32_t m, float32_t m_k){
	return v + (x - m)*(x - m_k);
}

void computeGyroStats(float32_t gyro[3], float32_t accel[3], float32_t mag[3], float32_t gyro_var[3], float32_t accel_var[3], float32_t mag_var[3]){
	static float32_t m_g[3] = {0};
	static float32_t v_g[3] = {0};
	static float32_t m_a[3] = {0};
	static float32_t v_a[3] = {0};
	static float32_t m_m[3] = {0};
	static float32_t v_m[3] = {0};

	static uint32_t count = 0;
	static bool first_enter = true;
	int i;

	if (first_enter){
		for (i = 0; i < 3; i++){
			v_g[i] = 0.0f;
			m_g[i] = gyro[i];
			v_a[i] = 0.0f;
			m_a[i] = accel[i];
			v_m[i] = 0.0f;
			m_m[i] = mag[i];
		}

		count++;
		first_enter = false;
		return;
	}

	float32_t m_k;
	for (i = 0; i < 3; i++){
		m_k = computeMean(m_g[i], gyro[i], (float32_t)count);
		v_g[i] = computeVariance(v_g[i], gyro[i], m_g[i], m_k);
		m_g[i] = m_k;

		m_k = computeMean(m_a[i], accel[i], (float32_t)count);
		v_a[i] = computeVariance(v_a[i], accel[i], m_a[i], m_k);
		m_a[i] = m_k;

		m_k = computeMean(m_m[i], mag[i], (float32_t)count);
		v_m[i] = computeVariance(v_m[i], mag[i], m_m[i], m_k);
		m_m[i] = m_k;

		count++;
		gyro_var[i] = v_g[i]/((float32_t)count - 1.0f);
		accel_var[i] = v_a[i]/((float32_t)count - 1.0f);
		mag_var[i] = v_m[i]/((float32_t)count - 1.0f);
	}
}

int main(void) {
	float32_t average_imu[6] = {0};
	float32_t w[3] = {0};
	float32_t q[4] = {0};
	// enable FPU full access
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

	SysTick_Config(SystemCoreClock/1000);
	init_I2C1(); // initialize I2C peripheral
	MPU6050_Initialize();
	//init_gyro(I2C1);
	//init_accel(I2C1);
	init_mag(I2C1);
	init_gpio();
	init_USART2(115200);
	init_cci(129);
	//computeInitMeasurementFrame(average_imu);
	//init_ekf(average_imu);

	imu_data_s imu_data;
	float32_t ypr[3] = {0};
	int8_t temperature;
	char buffer[128];
	while (1) {
		if (flag10ms){
			//read_gyro(I2C1, imu_data.rate, &temperature);
			//read_accel(I2C1, imu_data.acceleration);
			read_mag(I2C1, imu_data.magnetic);	// uT
			imu_data.time = (float)SysTickCounter*1.0e-3f;
			//run_ekf(0.1, imu_data.rate, imu_data.acceleration, imu_data.magnetic, &q[0], &w[0]);
			//triadComputation(&average_imu[0], &average_imu[3], imu_data.acceleration, imu_data.magnetic, ypr);
			/*

			 int len = sprintf(&buffer[0], "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
					imu_data.time,
					imu_data.rate[0], imu_data.rate[1], imu_data.rate[2],
					imu_data.magnetic[0],imu_data.magnetic[1],imu_data.magnetic[2]);
*/
			int len = sprintf(&buffer[0], "%.3f,%.3f,%.3f\r\n",ypr[0],ypr[1],ypr[2]);
			//USART_sendInt(&imu_data, sizeof(imu_data_s));
			//USART_send(USART2, &imu_data, sizeof(imu_data_s));
			USART_send(USART2, &buffer[0], len);
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			flag10ms = false;
		}
/*
		if (flag100ms){
			int len = sprintf(&buffer[0], "%8.3f:%8.2f,%8.2f,%8.2f|%8.2f,%8.2f,%8.2f\r\n",
					imu_data.time,
					imu_data.rate[0], imu_data.rate[1], imu_data.rate[2],
					imu_data.magnetic[0],imu_data.magnetic[1],imu_data.magnetic[2]);
			USART_sendInt(&buffer[0], len);
			GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			flag100ms = false;
		}
*/
	}
}
