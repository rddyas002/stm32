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
#include "ekf32.h"

#include <arm_math.h>

float32_t computeMean(float32_t m, float32_t x, float32_t k);
float32_t computeVariance(float32_t v, float32_t x, float32_t m, float32_t m_k);
void computeGyroStats(imu_data_s * imu_data);

__IO uint32_t SysTickCounter = 0;
__IO int32_t DelayCounter = 0;
__IO bool flag10ms = false;
__IO bool flag100ms = false;
void timingHandler(void){
	SysTickCounter++;

	if(!(SysTickCounter % 10))
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

/*
 * USART: PA9 [Tx]  PA10 [Rx]
 * I2C:	  PB6 [SCL] PB7 [SDA]
 * PWM:   PB4|PB5|PB0|PB1 --> |TIM3_CH1|TIM3_CH2|TIM3_CH3|TIM3_CH4|
 */
volatile uint32_t motors[4] = {PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO};
int main(void) {
	imu_data_s imu_data;
	uint8_t buffer[256];

	float32_t Q_diag[6] = {0.35e-5f,0.35e-5f,0.35e-5f,2.5e-8f,2.5e-8f,2.5e-8f};
	float32_t R_diag[6] = {0.3e-4f,0.3e-4f,0.3e-4f,0.4e-4f,0.4e-4f,0.4e-4f};

	float32_t b_32[3] = {0};
	float32_t q_32[4] = {0};
	float32_t ypr_32[3] = {0};
	float64_t b_64[3] = {0};
	float64_t q_64[4] = {0};
	float64_t ypr_64[3] = {0};

	// enable FPU full access
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */

	SysTick_Config(SystemCoreClock/1000);
	init_I2C1();
	MPU6050_Initialize();
	init_mag(I2C1);
	init_gpio();
	init_USART2(115200);
	init_cci(129);
	init_tim4();
	computeInitMeasurementFrame(&imu_data);

	init_ekf(&imu_data);
	init_ekf_32(&imu_data);

	float32_t prev_time = 0;
	float32_t delta_t_32 = 0;
	float64_t delta_t_64 = 0;

	while (1) {
		if (flag10ms){
			// read sensors
			MPU6050_GetRawAccelGyro(&imu_data);
			read_mag(I2C1, imu_data.magnetic);
			// calculate dt
			imu_data.time = (float)SysTickCounter*1.0e-3;
			if (prev_time == 0){
				delta_t_32 = 10e-3f;
				delta_t_64 = 10e-3;
			}
			else{
				delta_t_32 = imu_data.time - prev_time;
				delta_t_64 = (double)delta_t_32;
			}
			prev_time = imu_data.time;

			GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
			run_ekf_32(delta_t_32, imu_data.rate, imu_data.acceleration, imu_data.magnetic, &q_32[0], &b_32[0]);
			q2ypr_32(q_32, ypr_32);
			float w_est[3] = {0};
			w_est[0] = imu_data.rate[0];	//dps
			w_est[1] = imu_data.rate[1];	//dps
			w_est[2] = imu_data.rate[2];	//dps
			GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);

			w_est[0] *= 680/(2000*1.22); // maps -2000<->+2000 to -557<->+557
			w_est[1] *= 680/(2000*1.22);
			w_est[2] *= 680/(2000*1.22);

			// motors should range between -680<->+680
			motors[0] =  0.0000*w_est[0] + 0.7500*w_est[1] + 0.0000*w_est[2];
			motors[1] =  0.7071*w_est[0] + 0.2500*w_est[1] + 0.0000*w_est[2];
			motors[2] = -0.3536*w_est[0] + 0.2500*w_est[1] + 0.6124*w_est[2];
			motors[3] = -0.3536*w_est[0] + 0.2500*w_est[1] - 0.6124*w_est[2];

			flag10ms = false;
		}

		if (flag100ms){
			int len = sprintf((char *)&buffer[0], "%5.2f,%5.2f,%5.2f|%5.2f,%5.2f,%5.2f\r\n",ypr_32[0],ypr_32[1],ypr_32[2],b_32[0],b_32[1],b_32[2]);
			//USART_send(USART2, &buffer[0], len);
			USART_sendInt(&buffer[0], len);
			flag100ms = false;
		}

	}
}

void TIM3_IRQHandler(void)
{
if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    TIM_SetCompare1(TIM3, PWM_sat_limit(motors[0] + PWM_ZERO));
    TIM_SetCompare2(TIM3, PWM_sat_limit(motors[1] + PWM_ZERO));
    TIM_SetCompare3(TIM3, PWM_sat_limit(motors[2] + PWM_ZERO));
    TIM_SetCompare4(TIM3, PWM_sat_limit(motors[3] + PWM_ZERO));

  }
}
