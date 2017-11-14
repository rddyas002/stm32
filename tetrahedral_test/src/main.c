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
void computeGyroStats(imu_data_s * imu_data);

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

void computeInitMeasurementFrame(imu_data_s * imu_data){
	int i;
	double measurements[9] = {0};

	for (i = 0; i < 50; i++){
		MPU6050_GetRawAccelGyro(imu_data);
		read_mag(I2C1, imu_data->magnetic);
		computeGyroStats(imu_data);
		// store data
		measurements[0] += (double)imu_data->acceleration[0];
		measurements[1] += (double)imu_data->acceleration[1];
		measurements[2] += (double)imu_data->acceleration[2];
		measurements[3] += (double)imu_data->rate[0];
		measurements[4] += (double)imu_data->rate[1];
		measurements[5] += (double)imu_data->rate[2];
		measurements[6] += (double)imu_data->magnetic[0];
		measurements[7] += (double)imu_data->magnetic[1];
		measurements[8] += (double)imu_data->magnetic[2];
		msDelaySysTick(50);
	}

	for (i = 0; i < 9; i++)
		measurements[i] /= 50.0;

	imu_data->accel_offset[0] = (float32_t)measurements[0];
	imu_data->accel_offset[1] = (float32_t)measurements[1];
	imu_data->accel_offset[2] = (float32_t)measurements[2];

	imu_data->gyro_offset[0] = (float32_t)measurements[3];
	imu_data->gyro_offset[1] = (float32_t)measurements[4];
	imu_data->gyro_offset[2] = (float32_t)measurements[5];

	imu_data->mag_offset[0] = (float32_t)measurements[6];
	imu_data->mag_offset[1] = (float32_t)measurements[7];
	imu_data->mag_offset[2] = (float32_t)measurements[8];

}

float32_t computeMean(float32_t m, float32_t x, float32_t k){
	return m + (x - m)/k;
}

float32_t computeVariance(float32_t v, float32_t x, float32_t m, float32_t m_k){
	return v + (x - m)*(x - m_k);
}

void computeGyroStats(imu_data_s * imu_data){
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
	for (i = 0; i < 3; i++){
		m_k = computeMean(m_g[i], imu_data->rate[i], (float32_t)count);
		v_g[i] = computeVariance(v_g[i], imu_data->rate[i], m_g[i], m_k);
		m_g[i] = m_k;

		m_k = computeMean(m_a[i], imu_data->acceleration[i], (float32_t)count);
		v_a[i] = computeVariance(v_a[i], imu_data->acceleration[i], m_a[i], m_k);
		m_a[i] = m_k;

		m_k = computeMean(m_m[i], imu_data->magnetic[i], (float32_t)count);
		v_m[i] = computeVariance(v_m[i], imu_data->magnetic[i], m_m[i], m_k);
		m_m[i] = m_k;

		count++;
		imu_data->gyro_var[i] = v_g[i]/((float32_t)count - 1.0f);
		imu_data->accel_var[i] = v_a[i]/((float32_t)count - 1.0f);
		imu_data->mag_var[i] = v_m[i]/((float32_t)count - 1.0f);
	}
}

int main(void) {
	imu_data_s imu_data;
	float32_t ypr[3] = {0};
	char buffer[128];

	float32_t w[3] = {0};
	float32_t q[4] = {0};

	// enable FPU full access
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

	SysTick_Config(SystemCoreClock/1000);
	init_I2C1();
	MPU6050_Initialize();
	init_mag(I2C1);
	init_gpio();
	init_USART2(115200);
	init_cci(129);
	computeInitMeasurementFrame(&imu_data);
	init_ekf(&imu_data);
	float32_t prev_time = 0;
	float32_t delta_t = 0;
	while (1) {
		if (flag10ms){
			MPU6050_GetRawAccelGyro(&imu_data);
			read_mag(I2C1, imu_data.magnetic);	// uT
			imu_data.time = (float)SysTickCounter*1.0e-3f;
			if (prev_time == 0){
				delta_t = 20e-3f;
				prev_time = imu_data.time;
			}
			else{
				delta_t = imu_data.time - prev_time;
				prev_time = imu_data.time;
			}
			delta_t = 20e-3f;
			imu_data.rate[0] = -0.0320f;
			imu_data.rate[1] = 0.0220f;
			imu_data.rate[2] = -0.0110f;
			imu_data.acceleration[0] = 0.001f;
			imu_data.acceleration[1] = 0.015f;
			imu_data.acceleration[2] = -1.00f;
			imu_data.magnetic[0] = 0.124f;
			imu_data.magnetic[1] = -0.658f;
			imu_data.magnetic[2] = 0.7430f;

			run_ekf(delta_t, imu_data.rate, imu_data.acceleration, imu_data.magnetic, &q[0], &w[0]);
//			triadComputation(imu_data.accel_offset, imu_data.mag_offset, imu_data.acceleration, imu_data.magnetic, ypr);
/*
			 int len = sprintf(&buffer[0], "%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f\r\n",
					imu_data.time,
					imu_data.rate[0], imu_data.rate[1], imu_data.rate[2],
					imu_data.acceleration[0], imu_data.acceleration[1], imu_data.acceleration[2],
					imu_data.magnetic[0],imu_data.magnetic[1],imu_data.magnetic[2]);
					*/
/*
			 int len = sprintf(&buffer[0], "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
					imu_data.time,
					imu_data.rate[0], imu_data.rate[1], imu_data.rate[2],
					imu_data.acceleration[0], imu_data.acceleration[1], imu_data.acceleration[2],
					imu_data.magnetic[0],imu_data.magnetic[1],imu_data.magnetic[2]);
*/
			//q2ypr(q, ypr);
			//int len = sprintf(&buffer[0], "%5.2f,%5.2f,%5.2f\r\n",ypr[0],ypr[1],ypr[2]);

			int len = sprintf(&buffer[0], "%5.2f,%5.2f,%5.2f,%5.2f|%5.2f,%5.2f,%5.2f|%5.2f,%5.2f,%5.2f|\r\n",
					q[0],q[1],q[2],q[3],
					w[0]*180.0f/M_PI_f,w[1]*180.0f/M_PI_f,w[2]*180.0f/M_PI_f,
					imu_data.rate[0]*180.0f/M_PI_f,imu_data.rate[1]*180.0f/M_PI_f,imu_data.rate[2]*180.0f/M_PI_f);

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
