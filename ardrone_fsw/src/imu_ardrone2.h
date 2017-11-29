#ifndef IMU_ARDRONE2_H_
#define IMU_ARDRONE2_H_

#define IMU_GYRO_SENS	16.4
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   -1
#define IMU_GYRO_R_SIGN   -1

/** default accel sensitivy from the datasheet
 * 512 LSB/g
 */
#define IMU_ACCEL_SENS	512
#define IMU_ACCEL_X_SIGN   -1
#define IMU_ACCEL_Y_SIGN   1
#define IMU_ACCEL_Z_SIGN   1

#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 2048
#define IMU_ACCEL_Y_NEUTRAL 2048
#define IMU_ACCEL_Z_NEUTRAL 2048
#endif

#if !defined IMU_MAG_X_SENS & !defined IMU_MAG_Y_SENS & !defined IMU_MAG_Z_SENS
#define IMU_MAG_X_SENS 16.0
#define IMU_MAG_X_SENS_NUM 16
#define IMU_MAG_X_SENS_DEN 1
#define IMU_MAG_Y_SENS 16.0
#define IMU_MAG_Y_SENS_NUM 16
#define IMU_MAG_Y_SENS_DEN 1
#define IMU_MAG_Z_SENS 16.0
#define IMU_MAG_Z_SENS_NUM 16
#define IMU_MAG_Z_SENS_DEN 1
#endif

#endif /* IMU_ARDRONE2_H_ */
